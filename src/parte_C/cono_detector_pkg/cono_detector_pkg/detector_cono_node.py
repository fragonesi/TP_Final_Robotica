import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan

# Umbrales HSV para rojo (el rojo cruza el 0/180 en el espacio de Hue de OpenCV,
# por eso se necesitan dos rangos y se unen con un OR).
HSV_ROJO_BAJO_1 = (0, 120, 70)
HSV_ROJO_ALTO_1 = (10, 255, 255)
HSV_ROJO_BAJO_2 = (170, 120, 70)
HSV_ROJO_ALTO_2 = (180, 255, 255)

AREA_MINIMA_PX = 300       # contornos mas chicos que esto se descartan como ruido
VENTANA_LIDAR = 3          # rayos a cada lado del rayo objetivo para tomar el minimo
OFFSET_LIDAR_TB4 = 90.0    # mismo offset usado en tp0.py: indice 0 del lidar no apunta al frente del robot
RANGO_MAXIMO_VALIDO = 8.0  # descarta retornos de lidar absurdamente lejanos


class DetectorConoNode(Node):
    def __init__(self):
        super().__init__('detector_cono_node')

        self.declare_parameter('robot', 'tb4')
        self.declare_parameter('image_topic', '/tb4_0/oakd/rgb/preview/image_raw')
        self.declare_parameter('camera_info_topic', '/tb4_0/oakd/rgb/preview/camera_info')
        self.declare_parameter('scan_topic', '/tb4_0/scan')
        self.declare_parameter('output_topic', '/tb4_0/cono_detectado')
        self.declare_parameter('frame_id', 'tb4_0/base_link')
        self.declare_parameter('publicar_debug_image', True)
        # fx/cx de respaldo por si camera_info todavia no publico nada (se pisan apenas llega el primer mensaje)
        self.declare_parameter('fx_fallback', 600.0)
        self.declare_parameter('cx_fallback', 320.0)

        robot = self.get_parameter('robot').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publicar_debug = self.get_parameter('publicar_debug_image').get_parameter_value().bool_value

        self.fx = self.get_parameter('fx_fallback').get_parameter_value().double_value
        self.cx = self.get_parameter('cx_fallback').get_parameter_value().double_value
        self.offset_lidar = OFFSET_LIDAR_TB4 if robot == 'tb4' else 0.0

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.bridge = CvBridge()
        self.last_scan = None

        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, scan_qos)

        self.punto_pub = self.create_publisher(PointStamped, output_topic, 10)
        if self.publicar_debug:
            self.debug_pub = self.create_publisher(Image, output_topic + '/debug_image', 10)

        self.get_logger().info(f'Detector de conos iniciado (robot={robot}, img={image_topic}, scan={scan_topic})')

    def camera_info_callback(self, msg: CameraInfo):
        # K = [fx 0 cx; 0 fy cy; 0 0 1] -> me interesa solo el eje horizontal para el bearing
        self.fx = msg.k[0]
        self.cx = msg.k[2]

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        centroide = self.detectar_cono(frame)

        if centroide is None:
            return

        u, v = centroide
        bearing = self.pixel_a_bearing(u)
        distancia = self.distancia_por_lidar(bearing)

        if distancia is None:
            self.get_logger().info('Cono visible pero sin retorno de lidar valido en esa direccion')
            return

        self.publicar_deteccion(bearing, distancia, msg)

    def detectar_cono(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mascara_1 = cv2.inRange(hsv, HSV_ROJO_BAJO_1, HSV_ROJO_ALTO_1)
        mascara_2 = cv2.inRange(hsv, HSV_ROJO_BAJO_2, HSV_ROJO_ALTO_2)
        mascara = cv2.bitwise_or(mascara_1, mascara_2)

        kernel = np.ones((5, 5), np.uint8)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)

        contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contornos:
            self.ultima_mascara = mascara
            return None

        mayor = max(contornos, key=cv2.contourArea)
        if cv2.contourArea(mayor) < AREA_MINIMA_PX:
            self.ultima_mascara = mascara
            return None

        m = cv2.moments(mayor)
        u = m['m10'] / m['m00']
        v = m['m01'] / m['m00']

        if self.publicar_debug:
            self.publicar_imagen_debug(frame, mayor, (int(u), int(v)))

        return u, v

    def pixel_a_bearing(self, u):
        # Modelo pinhole: angulo entre el rayo optico y el rayo hacia el pixel detectado.
        # u < cx (objeto a la izquierda en la imagen) -> bearing positivo (izquierda en convencion ROS, REP103)
        return math.atan2(self.cx - u, self.fx)

    def distancia_por_lidar(self, bearing):
        scan = self.last_scan
        if scan is None:
            return None

        n = len(scan.ranges)
        if n == 0:
            return None

        # indice del rayo que apunta al frente del robot, corrigiendo el offset de montaje del lidar
        idx_frente = int((self.offset_lidar / 360.0) * n)
        # cuantos rayos hay que correrse desde el frente para llegar al bearing del cono
        idx_objetivo = idx_frente + int(round(bearing / scan.angle_increment))

        validos = []
        for i in range(idx_objetivo - VENTANA_LIDAR, idx_objetivo + VENTANA_LIDAR + 1):
            r = scan.ranges[i % n]
            if math.isfinite(r) and 0.0 < r <= RANGO_MAXIMO_VALIDO:
                validos.append(r)

        if not validos:
            return None
        return min(validos)

    def publicar_deteccion(self, bearing, distancia, img_msg: Image):
        punto = PointStamped()
        punto.header.stamp = img_msg.header.stamp
        punto.header.frame_id = self.frame_id
        # convencion: x adelante, y izquierda (REP103)
        punto.point.x = distancia * math.cos(bearing)
        punto.point.y = distancia * math.sin(bearing)
        punto.point.z = 0.0
        self.punto_pub.publish(punto)
        self.get_logger().info(
            f'Cono detectado: x={punto.point.x:.2f} y={punto.point.y:.2f} (bearing={math.degrees(bearing):.1f} deg)'
        )

    def publicar_imagen_debug(self, frame, contorno, centro):
        debug = frame.copy()
        cv2.drawContours(debug, [contorno], -1, (0, 255, 0), 2)
        cv2.circle(debug, centro, 5, (255, 0, 0), -1)
        msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        self.debug_pub.publish(msg)


def main():
    rclpy.init()
    node = DetectorConoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
