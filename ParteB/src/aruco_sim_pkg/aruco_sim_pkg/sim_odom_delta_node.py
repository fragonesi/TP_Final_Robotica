"""Odometria de Gazebo -> deltas de movimiento (modelo de Thrun).

Para Sistema 3 (Parte B). Port casi literal de aruco_pkg/odom_delta_node.py
(mismo modelo, mismo formato de CSV), con dos diferencias respecto del
original de Parte A:

  - Se suscribe a `/odom` (el diff-drive plugin del burger en Gazebo), no a
    `/tb4_0/odom` del bag real.
  - QoS por defecto (RELIABLE) en vez de BEST_EFFORT: ese gotcha es
    especifico de como el bag real publica sus tópicos; los tópicos de
    Gazebo en simulacion se publican RELIABLE como corresponde al default
    de ROS 2, asi que forzar BEST_EFFORT aca no tendria motivo y podria
    incluso perder mensajes de mas.

Salida: CSV `log_csv_path` (timestamp, x, y, theta, delta_rot1, delta_trans,
delta_rot2, dt), mismo formato que `odom_deltas.csv` de Parte A, para que
`slam_pipeline.py`/`graph_slam.py` lo consuman sin modificar.
"""
import csv
import math
import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SimOdomDeltaNode(Node):
    def __init__(self):
        super().__init__('sim_odom_delta_node')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('log_csv_path', 'odom_deltas_sim.csv')
        self.declare_parameter('min_trans_for_rot1', 0.01)

        odom_topic = self.get_parameter('odom_topic').value
        self.log_csv_path = self.get_parameter('log_csv_path').value
        self.min_trans_for_rot1 = float(self.get_parameter('min_trans_for_rot1').value)

        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None
        self.prev_t = None

        self._init_csv()
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.get_logger().info(f'sim_odom_delta_node listo. odom_topic={odom_topic}')

    def _init_csv(self):
        is_new = not os.path.isfile(self.log_csv_path)
        self.csv_file = open(self.log_csv_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if is_new:
            self.csv_writer.writerow(
                ['timestamp', 'x', 'y', 'theta', 'delta_rot1', 'delta_trans', 'delta_rot2', 'dt'])

    def _odom_cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = yaw_from_quaternion(msg.pose.pose.orientation)

        if self.prev_x is None:
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

        self.csv_writer.writerow([t, x, y, theta, delta_rot1, delta_trans, delta_rot2, dt])
        self.csv_file.flush()

        self.prev_x, self.prev_y, self.prev_theta, self.prev_t = x, y, theta, t

    def destroy_node(self):
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimOdomDeltaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
