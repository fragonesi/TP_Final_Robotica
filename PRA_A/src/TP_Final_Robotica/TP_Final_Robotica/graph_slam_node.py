"""Nodo ROS 2 que corre GraphSLAM (batch, desde CSV) y publica para RViz.

Carga los CSV de odometría/ArUco, optimiza una vez con graph_slam.py y publica
periódicamente los tópicos que la consigna pide visualizar:

    /belief          nav_msgs/Path                    trayectoria corregida
    /landmarks       visualization_msgs/MarkerArray   ArUco estimados, por ID
    /poses_guardadas geometry_msgs/PoseArray          nodos de pose del grafo

Se publica en un timer (no una sola vez) para que RViz tome los mensajes aunque
se conecte tarde.

Uso:
    ros2 run TP_Final_Robotica graph_slam_node --ros-args \
        -p odom_csv:=/ruta/odom_deltas.csv \
        -p aruco_csv:=/ruta/aruco_detections.csv
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from .graph_slam import build_from_csv


def yaw_to_quat(yaw):
    """(x, y, z, w) de una rotación pura en yaw."""
    return 0.0, 0.0, float(np.sin(yaw / 2.0)), float(np.cos(yaw / 2.0))


class GraphSlamNode(Node):
    def __init__(self):
        super().__init__('graph_slam_node')
        self.declare_parameter('odom_csv', '')
        self.declare_parameter('aruco_csv', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('iterations', 30)
        self.declare_parameter('noise_model_path', '')

        odom = self.get_parameter('odom_csv').value
        aruco = self.get_parameter('aruco_csv').value or None
        self.frame_id = self.get_parameter('frame_id').value
        iters = int(self.get_parameter('iterations').value)
        nm_path = self.get_parameter('noise_model_path').value or None
        if nm_path is None:
            # Auto-detectar desde share/aruco_pkg/ si se instaló con colcon build.
            try:
                from ament_index_python.packages import get_package_share_directory
                candidate = os.path.join(
                    get_package_share_directory('aruco_pkg'), 'noise_model.json')
                if os.path.isfile(candidate):
                    nm_path = candidate
            except Exception:
                pass
        if nm_path:
            self.get_logger().info(f'Modelo de ruido ArUco: {nm_path}')
        else:
            self.get_logger().warn('Sin noise_model.json — usando covarianza ArUco por defecto')

        self.belief_pub = self.create_publisher(Path, '/belief', 10)
        self.lm_pub = self.create_publisher(MarkerArray, '/landmarks', 10)
        self.poses_pub = self.create_publisher(PoseArray, '/poses_guardadas', 10)

        if not odom:
            self.get_logger().error('Falta -p odom_csv:=/ruta/al/odom_deltas.csv')
            return

        self.get_logger().info(f'Optimizando GraphSLAM desde {odom} ...')
        self.g = build_from_csv(odom, aruco, noise_model_path=nm_path)
        self.g.optimize(iterations=iters, verbose=False)
        self.get_logger().info(
            f'Listo: {self.g.n_poses} poses, {len(self.g.landmark_ids)} landmarks. '
            f'Publicando en RViz (frame={self.frame_id}).')

        self.create_timer(1.0, self.publish_all)

    def publish_all(self):
        stamp = self.get_clock().now().to_msg()

        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = stamp
        poses = PoseArray()
        poses.header = path.header

        for p in self.g.poses:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            _, _, qz, qw = yaw_to_quat(p[2])
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
            poses.poses.append(ps.pose)

        self.belief_pub.publish(path)
        self.poses_pub.publish(poses)

        markers = MarkerArray()
        for lid in self.g.landmark_ids:
            lx, ly = self.g.landmarks[lid]
            m = Marker()
            m.header = path.header
            m.ns = 'landmarks'
            m.id = int(lid)
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(lx)
            m.pose.position.y = float(ly)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.3
            m.scale.z = 0.5
            m.color.r, m.color.g, m.color.a = 1.0, 0.6, 1.0
            markers.markers.append(m)

            label = Marker()
            label.header = path.header
            label.ns = 'landmark_ids'
            label.id = int(lid)
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(lx)
            label.pose.position.y = float(ly)
            label.pose.position.z = 0.6
            label.pose.orientation.w = 1.0
            label.scale.z = 0.3
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'id {lid}'
            markers.markers.append(label)

        self.lm_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = GraphSlamNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
