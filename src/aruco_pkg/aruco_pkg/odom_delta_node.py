import csv
import math
import os

import rclpy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def yaw_from_quaternion(q):
    """Extrae el yaw (rotación en Z) de un quaternion geometry_msgs/Quaternion.
    Asume robot en 2D (roll y pitch ~0), válido para TurtleBot4 en piso plano.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Lleva un ángulo a [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class OdomDeltaNode(Node):
    def __init__(self):
        super().__init__('odom_delta_node')

        self.declare_parameter('odom_topic', '/tb4_0/odom')
        self.declare_parameter('log_csv_path', 'odom_deltas.csv')
        self.declare_parameter('min_trans_for_rot1', 0.01)  # metros

        odom_topic = self.get_parameter('odom_topic').value
        self.log_csv_path = self.get_parameter('log_csv_path').value
        # Si el desplazamiento entre dos poses es casi nulo, atan2(dy,dx) es
        # ruido puro (el robot gira "in place" o está quieto); por debajo de
        # este umbral, delta_rot1 se fuerza a 0 y toda la rotación se carga
        # a delta_rot2, para no inyectar ruido artificial al grafo.
        self.min_trans_for_rot1 = float(self.get_parameter('min_trans_for_rot1').value)

        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None
        self.prev_t = None

        self.deltas_pub = self.create_publisher(Vector3, '/odom_deltas', 10)

        self._init_csv()

        # El tópico de odom en el bag se publicó con QoS BEST_EFFORT (típico
        # en datos de sensores reales). Si la suscripción pide RELIABLE
        # (el default de ROS 2), ROS 2 las considera incompatibles y no
        # entrega NINGÚN mensaje, sin error explícito más que un WARN.
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Odometry, odom_topic, self.odom_cb, odom_qos)

        self.get_logger().info(f'odom_delta_node listo. odom_topic={odom_topic}')

    def _init_csv(self):
        is_new = not os.path.isfile(self.log_csv_path)
        self.csv_file = open(self.log_csv_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if is_new:
            self.csv_writer.writerow(
                ['timestamp', 'x', 'y', 'theta',
                 'delta_rot1', 'delta_trans', 'delta_rot2', 'dt']
            )

    def odom_cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = yaw_from_quaternion(msg.pose.pose.orientation)

        if self.prev_x is None:
            # Primera lectura: no hay pose anterior para calcular delta.
            self.prev_x, self.prev_y, self.prev_theta, self.prev_t = x, y, theta, t
            return

        dx = x - self.prev_x
        dy = y - self.prev_y
        dt = t - self.prev_t

        delta_trans = math.hypot(dx, dy)

        if delta_trans < self.min_trans_for_rot1:
            delta_rot1 = 0.0
        else:
            delta_rot1 = normalize_angle(math.atan2(dy, dx) - self.prev_theta)

        delta_rot2 = normalize_angle(theta - self.prev_theta - delta_rot1)

        msg_out = Vector3()
        msg_out.x = delta_rot1
        msg_out.y = delta_trans
        msg_out.z = delta_rot2
        self.deltas_pub.publish(msg_out)

        self.csv_writer.writerow([t, x, y, theta, delta_rot1, delta_trans, delta_rot2, dt])
        self.csv_file.flush()

        self.get_logger().info(
            f'delta_rot1={delta_rot1:.4f} rad, delta_trans={delta_trans:.4f} m, '
            f'delta_rot2={delta_rot2:.4f} rad, dt={dt:.3f} s',
            throttle_duration_sec=1.0
        )

        self.prev_x, self.prev_y, self.prev_theta, self.prev_t = x, y, theta, t

    def destroy_node(self):
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomDeltaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()