"""Guarda a CSV los barridos LIDAR de Gazebo, para la 2da pasada de Sistema 3.

Port casi literal de TP_Final_Robotica/scan_logger_node.py (grilla de
ocupacion): mismo formato de CSV, pero suscripto a `/scan` con QoS por
defecto (RELIABLE) en vez de BEST_EFFORT -- ese override es especifico de
como el bag real de Parte A publica sus tópicos; el LIDAR simulado del
burger en Gazebo se publica RELIABLE.
"""
import csv
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SimScanLoggerNode(Node):
    def __init__(self):
        super().__init__('sim_scan_logger_node')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('log_csv_path', 'scans_sim.csv')
        self.declare_parameter('decimation', 1)

        scan_topic = self.get_parameter('scan_topic').value
        self.log_csv_path = self.get_parameter('log_csv_path').value
        self.decimation = max(1, int(self.get_parameter('decimation').value))

        self.csv_file = None
        self.csv_writer = None
        self.has_intensities = False
        self.count = 0

        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)
        self.get_logger().info(f'sim_scan_logger_node listo. scan_topic={scan_topic}')

    def _scan_cb(self, msg: LaserScan):
        self.count += 1
        if (self.count - 1) % self.decimation != 0:
            return
        if self.csv_writer is None:
            n_beams = len(msg.ranges)
            self.has_intensities = len(msg.intensities) == n_beams
            is_new = not os.path.isfile(self.log_csv_path)
            self.csv_file = open(self.log_csv_path, 'a', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            if is_new:
                header = (['timestamp', 'angle_min', 'angle_increment',
                           'range_min', 'range_max']
                          + [f'r{i}' for i in range(n_beams)])
                if self.has_intensities:
                    header += [f'i{i}' for i in range(n_beams)]
                self.csv_writer.writerow(header)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        row = [t, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max]
        row.extend(msg.ranges)
        if self.has_intensities:
            row.extend(msg.intensities)
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimScanLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
