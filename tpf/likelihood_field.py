import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.ndimage import distance_transform_edt


class LikelihoodMapPublisher(Node):
    def __init__(self):
        super().__init__('likelihood_map_publisher')
        qos = rclpy.qos.QoSProfile(depth=1)
        qos.durability = rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(OccupancyGrid, '/likelihood_map', qos)
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos
        )

    def map_callback(self, msg):

        self.get_logger().info("Recibí mapa")
        prob_msg = OccupancyGrid()
        prob_msg.header = msg.header
        prob_msg.info = msg.info

        width = msg.info.width
        height = msg.info.height
        
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        free = (grid == 0)
        distances = distance_transform_edt(free) * msg.info.resolution #calculo la distancia a la celda ocupada mas cercana para cada celda libre

        sigma = 5.0
        likelihood = np.exp(-distances**2 / (2 * sigma**2))

        prob_msg.data = (likelihood*100).astype(np.int8).flatten().tolist()

        self.pub.publish(prob_msg)
        
        #PRUEBA

        self.get_logger().info("Published likelihood map")


def main(args=None):
    rclpy.init(args=args)
    node = LikelihoodMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()