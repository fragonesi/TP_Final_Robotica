#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class CloseScanVisualizer(Node):
    def __init__(self):
        super().__init__('close_scan_visualizer')

        self.declare_parameter('danger_radius', 0.4)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('marker_topic', '/close_obstacle_points')

        self.danger_radius = self.get_parameter('danger_radius').value
        scan_topic = self.get_parameter('scan_topic').value
        marker_topic = self.get_parameter('marker_topic').value

        self.sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10
        )
        self.pub = self.create_publisher(Marker, marker_topic, 10)

        self.get_logger().info(
            f'Escuchando {scan_topic} | marcando puntos < {self.danger_radius} m '
            f'| publicando en {marker_topic}'
        )

    def scan_callback(self, msg: LaserScan):
        marker = Marker()
        marker.header = msg.header  # mismo frame que el láser (ej: base_scan / laser_link)
        marker.ns = 'close_points'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        # lifetime 0 = se sobreescribe con cada scan nuevo, no acumula basura vieja
        marker.lifetime.sec = 0

        angle = msg.angle_min
        for r in msg.ranges:
            # descarta inf/nan y rangos fuera de los límites válidos del sensor
            if msg.range_min < r < min(msg.range_max, self.danger_radius):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                marker.points.append(Point(x=x, y=y, z=0.0))
            angle += msg.angle_increment

        self.pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = CloseScanVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()