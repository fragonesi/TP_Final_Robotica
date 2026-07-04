"""
Localization node for the real TurtleBot4 (Part C).
Identical to localization_node.py but subscribed to:
    /tb4_0/odom  (instead of /calc_odom)  with QoS BEST_EFFORT
    /tb4_0/scan  (instead of /scan)       with QoS BEST_EFFORT

The rest of the logic (particles, likelihood field, publications) remains unchanged.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (
    PoseArray, Pose, PoseWithCovarianceStamped, Quaternion, PoseStamped
)
from tf2_ros import TransformBroadcaster
from .particle_filter import RobotFunctions
from geometry_msgs.msg import TransformStamped

def yaw_to_quaternion(yaw):
    """
    Converts a yaw angle (in radians) to a Quaternion message.
    """
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


def get_yaw(q):
    """
    Extracts the yaw angle (in radians) from a Quaternion message.
    """
    return np.arctan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class LocalizationNodeTB4(Node):

    def __init__(self):
        super().__init__('localization_node')
        self.pf = RobotFunctions(num_particles=200)
        self.map = None
        self.likelihood = None
        self.last_calc_odom = None
        self.initialized = False

        qos_map = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        qos_be  = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_map)
        self.create_subscription(OccupancyGrid, '/likelihood_map', self.likelihood_callback, qos_map)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)

        # Topics específicos del TB4 con QoS BEST_EFFORT
        self.create_subscription(Odometry, '/tb4_0/odom', self.odom_callback, qos_be)
        self.create_subscription(LaserScan, '/tb4_0/scan', self.scan_callback, qos_be)

        self.belief_pub = self.create_publisher(PoseArray, '/belief', 10)
        self.estimated_pose_pub = self.create_publisher(PoseStamped, '/estimated_pose', 10)
        self.best_particle_pub = self.create_publisher(PoseStamped, '/best_particle', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            'LocalizationNodeTB4 iniciado: '
            '/tb4_0/odom y /tb4_0/scan (BEST_EFFORT)')

    def map_callback(self, msg):
        """
        Callback for receiving the occupancy grid map.
        """
        self.map = msg
        self.get_logger().info('Mapa recibido')

    def likelihood_callback(self, msg):
        """
        Callback for receiving the likelihood field map.
        """
        h = msg.info.height
        w = msg.info.width
        self.likelihood = np.array(msg.data).reshape(h, w)
        self.get_logger().info('Likelihood recibido')

    def initialpose_callback(self, msg):
        """
        Callback for receiving the initial pose estimate.
        Initializes the particle filter around the given pose with some noise.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = get_yaw(msg.pose.pose.orientation)
        for p in self.pf.particles:
            p.set(np.random.normal(x, 0.05),
                  np.random.normal(y, 0.05),
                  np.random.normal(theta, 0.05))
        self.initialized = True
        self.get_logger().info(
            f'Inicialización: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')

    def odom_callback(self, msg):
        """
        Callback for receiving odometry data.
        Computes the change in position and orientation since the last odometry message,
        and moves the particles accordingly.
        """
        if not self.initialized:
            return
        if self.last_calc_odom is None:
            self.last_calc_odom = msg
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        last_x = self.last_calc_odom.pose.pose.position.x
        last_y = self.last_calc_odom.pose.pose.position.y

        dx = x - last_x
        dy = y - last_y
        delta_t  = np.sqrt(dx**2 + dy**2)
        yaw = get_yaw(msg.pose.pose.orientation)
        last_yaw = get_yaw(self.last_calc_odom.pose.pose.orientation)

        MIN_TRANS = 0.01
        if delta_t < MIN_TRANS:
            delta_rot1 = 0.0
            delta_rot2 = yaw - last_yaw
        else:
            delta_rot1 = np.arctan2(dy, dx) - last_yaw
            delta_rot2 = yaw - last_yaw - delta_rot1

        delta_rot1 = np.arctan2(np.sin(delta_rot1), np.cos(delta_rot1))
        delta_rot2 = np.arctan2(np.sin(delta_rot2), np.cos(delta_rot2))

        self.pf.move_particles({'r1': delta_rot1, 'r2': delta_rot2, 't': delta_t})
        self.last_calc_odom = msg

    def scan_callback(self, msg):
        """
        Callback for receiving laser scan data.
        Updates the particle weights based on the scan and the likelihood field,
        and then publishes the belief (particles, estimated pose, best particle).
        """
        if not self.initialized:
            return
        if self.map is None or self.likelihood is None:
            return
        self.pf.update_particles(msg, self.map, self.likelihood)
        self.publish_belief()

    def publish_belief(self):
        """
        Publishes the current belief of the particle filter:
        - PoseArray of all particles
        - Estimated pose (mean of particles)
        - Best particle (highest weight)
        Also broadcasts the transform from 'map' to 'odom' based on the estimated pose.
        """
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for p in self.pf.particles:
            pose = Pose()
            pose.position.x = p.x
            pose.position.y = p.y
            pose.orientation = yaw_to_quaternion(p.orientation)
            msg.poses.append(pose)
        self.belief_pub.publish(msg)
        self._publish_estimated_pose()
        self._publish_best_particle()
        self._broadcast_map_to_odom()

    def _publish_estimated_pose(self):
        """
        Publishes the estimated pose of the robot based on the mean of the particles.
        """
        xs = [p.x for p in self.pf.particles]
        ys = [p.y for p in self.pf.particles]
        thetas = [p.orientation for p in self.pf.particles]
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(np.mean(xs))
        msg.pose.position.y = float(np.mean(ys))
        msg.pose.orientation = yaw_to_quaternion(
            float(np.arctan2(np.mean(np.sin(thetas)), np.mean(np.cos(thetas)))))
        self.estimated_pose_pub.publish(msg)

    def _publish_best_particle(self):
        """
        Publishes the best particle (highest weight) as a PoseStamped message.
        """
        if self.pf.best_particle is None:
            return
        bx, by, btheta = self.pf.best_particle
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(bx)
        msg.pose.position.y = float(by)
        msg.pose.orientation = yaw_to_quaternion(float(btheta))
        self.best_particle_pub.publish(msg)

    def _broadcast_map_to_odom(self):
        """
        Publishes the transform from 'map' to 'odom' based on the estimated pose of the robot.
        """
        if not self.pf.particles:
            return
        xs = [p.x for p in self.pf.particles]
        ys = [p.y for p in self.pf.particles]
        thetas = [p.orientation for p in self.pf.particles]
        x = float(np.mean(xs))
        y = float(np.mean(ys))
        yaw = float(np.arctan2(np.mean(np.sin(thetas)), np.mean(np.cos(thetas))))

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quaternion(yaw)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNodeTB4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
