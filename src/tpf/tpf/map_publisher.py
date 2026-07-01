import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from ament_index_python.packages import get_package_share_directory

import yaml
import numpy as np
from PIL import Image
import os


class MapPublisher(Node):

    def __init__(self):

        super().__init__('map_publisher')
        pkg_share = get_package_share_directory('tpf')
        map_yaml = os.path.join(pkg_share, 'map.yaml')

        with open(map_yaml, 'r') as f:
            map_config = yaml.safe_load(f)

        image_path = os.path.join(
            os.path.dirname(map_yaml),
            map_config['image']
        )
        img = Image.open(image_path)
        img = np.array(img)
        img = np.flipud(img)
        # img = np.fliplr(img)
        # img = np.rot90(img, k=2)
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = "map"
        self.map_msg.info.resolution = map_config['resolution']
        self.map_msg.info.width = img.shape[1]
        self.map_msg.info.height = img.shape[0]
        self.map_msg.info.origin.position.x = map_config['origin'][0]
        self.map_msg.info.origin.position.y = map_config['origin'][1]
        self.map_msg.info.origin.orientation.w = 1.0

        data = []
        for pixel in img.flatten():
            if pixel < 50:
                data.append(100)     # obstáculo
            else:
                data.append(0)       # todo lo demás es libre

        self.map_msg.data = data

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.pub = self.create_publisher(
            OccupancyGrid,
            "/map",
            qos
        )

        # self.timer = self.create_timer(
        #     1.0,
        #     self.publish_map
        # )
        
        self.timer = self.create_timer(0.5, self.publish_map_once)
        # self.publish_map()
        self.get_logger().info("Map publisher listo")

    def publish_map_once(self):
        self.publish_map()
        self.timer.cancel() 

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)

if __name__ == "__main__":
    main()