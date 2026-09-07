"""Publica /map (nav_msgs/OccupancyGrid) a partir del .pgm/.yaml exportado por
slam_pipeline.py / occupancy_grid.py, para verlo en RViz junto con /belief,
/landmarks y /poses_guardadas.

A diferencia del map_publisher de navegacion_pkg/despliegue_pkg (que tiene el
mapa fijo instalado en su share/), acá el mapa lo genera el propio pipeline de
la Parte A en un --out-dir arbitrario, así que la ruta se pasa por parámetro
(map_yaml_path) en vez de resolverse contra el share del paquete.

Lee el .pgm crudo (formato P5 que escribe occupancy_grid.export_ros_map:
0=ocupado, 254=libre, 205=desconocido, fila 0 = arriba) sin depender de PIL,
para no sumar dependencias nuevas al paquete.
"""
import os

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile


def _read_pgm(path):
    """Parser mínimo de PGM binario (P5), tal como lo escribe export_ros_map."""
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError(f'{path}: se esperaba PGM binario (P5), vino {magic!r}')
        # saltea comentarios '#'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        width, height = (int(v) for v in line.split())
        maxval = int(f.readline())
        data = np.frombuffer(f.read(width * height), dtype=np.uint8)
        if maxval > 255:
            raise ValueError(f'{path}: maxval {maxval} no soportado (se esperaba 255)')
        return data.reshape((height, width))


def _read_map_yaml(path):
    """Parser mínimo del .yaml de map_server (siempre generado por
    export_ros_map con este formato fijo: image/resolution/origin/negate/
    occupied_thresh/free_thresh), para no sumar una dependencia a PyYAML."""
    cfg = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or ':' not in line:
                continue
            key, _, value = line.partition(':')
            cfg[key.strip()] = value.strip()
    cfg['resolution'] = float(cfg['resolution'])
    cfg['origin'] = [float(v) for v in cfg['origin'].strip('[]').split(',')]
    cfg['negate'] = int(cfg.get('negate', 0))
    return cfg


class MapPublisherNode(Node):

    def __init__(self):
        super().__init__('slam_map_publisher')
        self.declare_parameter('map_yaml_path', '')
        map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value

        if not map_yaml_path or not os.path.isfile(map_yaml_path):
            self.get_logger().error(
                f"map_yaml_path='{map_yaml_path}' no existe. Pasar -p "
                "map_yaml_path:=/ruta/a/mapa.yaml (lo genera slam_pipeline.py). "
                "El nodo no publicará /map.")
            self.map_msg = None
            return

        cfg = _read_map_yaml(map_yaml_path)
        image_path = os.path.join(os.path.dirname(map_yaml_path), cfg['image'])
        img = _read_pgm(image_path)
        if cfg['negate']:
            img = 255 - img
        img = np.flipud(img)  # PGM: fila 0 arriba -> OccupancyGrid: fila 0 = origen (abajo)

        # export_ros_map (occupancy_grid.py) escribe siempre estos tres valores
        # literales -- 0/254/205 -- nunca grises intermedios, así que se
        # matchean directo en vez de re-derivar un umbral sobre occupied_thresh/
        # free_thresh (que además no corresponde 1:1 con el 205 "desconocido").
        data = np.full(img.shape, -1, dtype=np.int8)   # 205 -> desconocido
        data[img == 254] = 0                            # libre
        data[img == 0] = 100                             # ocupado

        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = 'map'
        self.map_msg.info.resolution = cfg['resolution']
        self.map_msg.info.width = img.shape[1]
        self.map_msg.info.height = img.shape[0]
        self.map_msg.info.origin.position.x = cfg['origin'][0]
        self.map_msg.info.origin.position.y = cfg['origin'][1]
        self.map_msg.info.origin.orientation.w = 1.0
        self.map_msg.data = data.flatten().tolist()

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OccupancyGrid, '/map', qos)
        # Periódico a 1 Hz además de TRANSIENT_LOCAL: RViz/nodos que arrancan
        # tarde reciben el mapa igual.
        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info(f'/map publicado desde {map_yaml_path}')

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    if node.map_msg is not None:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
