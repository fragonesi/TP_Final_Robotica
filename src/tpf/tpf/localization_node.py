import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped, Quaternion, PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from .particle_filter import RobotFunctions
import math


def yaw_to_quaternion(yaw):
    """Convert a yaw angle (in radians) into a Quaternion message."""
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q

def get_yaw(q):
    return np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization_node')
        self.pf = RobotFunctions(num_particles=200)
        self.map = None
        self.likelihood = None
        self.last_calc_odom = None
        
        qos_map = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_map)
        self.create_subscription(OccupancyGrid, '/likelihood_map', self.likelihood_callback, qos_map)     
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(Odometry, '/calc_odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.belief_pub = self.create_publisher(PoseArray, '/belief', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.initialized = False
        self.estimated_pose_pub = self.create_publisher(PoseStamped, '/estimated_pose', 10)

        #PROBAR
        self.best_particle_pub = self.create_publisher(PoseStamped, '/best_particle', 10)

    def map_callback(self,msg):
        self.map = msg
        self.get_logger().info("Mapa recibido")

    def likelihood_callback(self,msg):
        h = msg.info.height
        w = msg.info.width
        self.likelihood = np.array(msg.data).reshape(h,w)
        self.get_logger().info("Likelihood recibido")

    def initialpose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        theta = get_yaw(q)

        for p in self.pf.particles:
            p.set(np.random.normal(x, 0.05), np.random.normal(y, 0.05), np.random.normal(theta, 0.05))

        self.initialized = True

        self.get_logger().info(f'Inicialización: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')
        self.get_logger().info(f'Primera particula: {self.pf.particles[0].x}, {self.pf.particles[0].y}')

    def odom_callback(self,msg):
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

        delta_t = np.sqrt(dx**2 + dy**2)
        yaw = get_yaw(msg.pose.pose.orientation)
        last_yaw = get_yaw(self.last_calc_odom.pose.pose.orientation)

        MIN_TRANS = 0.01
        if delta_t < MIN_TRANS:
            delta_rot1 = 0.0
            delta_rot2 = yaw - last_yaw
        else:
            delta_rot1 = np.arctan2(dy, dx) - last_yaw
            delta_rot2 = yaw - last_yaw - delta_rot1

        #Normalizo a -pi, pi para evitar problemas al cruzar el límite +-pi
        delta_rot1 = np.arctan2(np.sin(delta_rot1), np.cos(delta_rot1))
        delta_rot2 = np.arctan2(np.sin(delta_rot2), np.cos(delta_rot2))

        odom = {'r1':delta_rot1, 'r2':delta_rot2, 't':delta_t}
        self.pf.move_particles(odom)
        self.last_calc_odom = msg

    def scan_callback(self,msg):
        if not self.initialized:
            self.get_logger().info("Esperando initialpose...")
            return
        # self.get_logger().info("Llegó scan")

        if self.map is None:
            self.get_logger().info("No hay mapa")
            return

        if self.likelihood is None:
            self.get_logger().info("No hay likelihood")
            return

        self.pf.update_particles(msg, self.map, self.likelihood)
        self.publish_belief()

    # def publish_estimated_pose(self):
    #     xs = [p.x for p in self.pf.particles]
    #     ys = [p.y for p in self.pf.particles]
    #     thetas = [p.orientation for p in self.pf.particles]

    #     mean_x = np.mean(xs)
    #     mean_y = np.mean(ys)
    #     mean_theta = np.arctan2(np.mean(np.sin(thetas)), np.mean(np.cos(thetas)))

    #     msg = PoseStamped()
    #     msg.header.frame_id = "map"
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.pose.position.x = mean_x
    #     msg.pose.position.y = mean_y
    #     msg.pose.orientation = yaw_to_quaternion(mean_theta)
    #     self.estimated_pose_pub.publish(msg)
    
    def publish_estimated_pose(self):
        weights = np.array([p.weight for p in self.pf.particles])
        total = weights.sum()
        if total < 1e-10:
            return
        weights = weights / total

        xs = np.array([p.x for p in self.pf.particles])
        ys = np.array([p.y for p in self.pf.particles])
        thetas = np.array([p.orientation for p in self.pf.particles])

        mean_x = float(np.sum(weights * xs))
        mean_y = float(np.sum(weights * ys))
        mean_theta = float(np.arctan2(
            np.sum(weights * np.sin(thetas)),
            np.sum(weights * np.cos(thetas))
        ))

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = mean_x
        msg.pose.position.y = mean_y
        msg.pose.orientation = yaw_to_quaternion(mean_theta)
        self.estimated_pose_pub.publish(msg)

    def publish_best_particle(self):
        if self.pf.best_particle is None:
            return
        bx, by, btheta = self.pf.best_particle
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(bx)
        msg.pose.position.y = float(by)
        msg.pose.orientation = yaw_to_quaternion(float(btheta))
        self.best_particle_pub.publish(msg)

    def publish_belief(self):
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for p in self.pf.particles:
            pose = Pose()
            pose.position.x = p.x
            pose.position.y = p.y
            pose.orientation = yaw_to_quaternion(p.orientation)
            msg.poses.append(pose)

        self.belief_pub.publish(msg)
        self.publish_estimated_pose()
        self.publish_best_particle()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()