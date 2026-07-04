"""Nodo ROS 2 que registra los barridos del LIDAR a CSV (para la 2da pasada).

Necesario para la grilla de ocupación: durante la reproducción del bag guarda cada
LaserScan, para luego proyectarlo con la trayectoria corregida por GraphSLAM.

OJO con el QoS: igual que la odometría, el LIDAR del bag se publica como BEST_EFFORT;
si la suscripción pide RELIABLE (default de ROS 2), no se entrega ningún mensaje.

Uso:
    ros2 run slam_pkg scan_logger_node --ros-args \
        -p scan_topic:=/tb4_0/scan -p log_csv_path:=scans.csv
"""
import csv
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanLoggerNode(Node):
    def __init__(self):
        super().__init__('scan_logger_node')
        self.declare_parameter('scan_topic', '/tb4_0/scan')
        self.declare_parameter('log_csv_path', 'scans.csv')
        self.declare_parameter('decimation', 1)   # guardar 1 de cada N barridos

        scan_topic = self.get_parameter('scan_topic').value
        self.log_csv_path = self.get_parameter('log_csv_path').value
        self.decimation = max(1, int(self.get_parameter('decimation').value))

        self.csv_file = None
        self.csv_writer = None
        self.count = 0

        # BEST_EFFORT: mismo motivo que en odom_delta_node (sensor real del bag).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(LaserScan, scan_topic, self.scan_cb, qos)
        self.get_logger().info(
            f'scan_logger_node listo. scan_topic={scan_topic} (QoS BEST_EFFORT)')

    def scan_cb(self, msg: LaserScan):
        self.count += 1
        if (self.count - 1) % self.decimation != 0:
            return
        if self.csv_writer is None:
            n_beams = len(msg.ranges)
            # El rplidar publica intensidades en paralelo a los rangos; las guardamos
            # (columnas i0..iN) para poder filtrar haces espurios (intensity==0) en la
            # 2da pasada — la parte0 de la cátedra descarta esos haces. Si el sensor no
            # las trae (intensities vacío), no agregamos columnas y el filtro queda nulo.
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
        if self.count % 100 == 0:
            self.get_logger().info(
                f'barridos guardados: {self.count // self.decimation}',
                throttle_duration_sec=2.0)

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScanLoggerNode()
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
