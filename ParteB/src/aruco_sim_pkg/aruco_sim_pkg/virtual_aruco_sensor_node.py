"""Sensor virtual de landmarks ArUco para Gazebo (Parte B, Sistema 3).

Como el burger de Gazebo no tiene camara y el mundo no tiene marcadores
visuales, este nodo emula la deteccion de ArUco de forma puramente
geometrica: conoce la pose "verdad-terreno" del robot (via
`/gazebo/model_states`, publicado por el plugin `gazebo_ros_state`) y la
pose de cada landmark virtual (YAML autorado, ver landmark_loader.py), y
para cada landmark decide si seria detectado (rango, FOV, orientacion del
marcador, oclusion) y con que ruido, replicando el comportamiento del
detector real de Parte A (aruco_detector_node.py) sin necesidad de imagen.

Por que verdad-terreno de Gazebo y no `/odom`: `/odom` ya arrastra el drift
que la propia Parte B busca corregir; si el sensor calculara rango/bearing
contra una pose ya degradada, estaria inyectando error de odometria
disfrazado de error de sensor. La camara real de Parte A tampoco "sabe"
de la odometria -- mide contra la geometria fisica real. Para reproducir
esa fidelidad, el sensor virtual necesita la pose real del simulador y
recien ENCIMA de eso aplica el modelo de ruido del ArUco.

Salida: publica `/aruco_detections` (aruco_sim_msgs/ArucoDetectionArray) y
loguea a CSV en el formato exacto de aruco_pkg (timestamp, marker_id, tx,
ty, tz, distancia_m, rx, ry, rz) para que `slam_pipeline.py`/`graph_slam.py`
lo consuman sin modificar (rx,ry,rz se logean en 0 porque esos campos
nunca se leen en `build_from_csv`).
"""
import csv
import json
import math
import os
import random

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray

from aruco_sim_msgs.msg import ArucoDetection, ArucoDetectionArray

from aruco_sim_pkg.landmark_loader import load_landmarks
from aruco_sim_pkg.occlusion import get_obstacles, is_occluded

# Fallback si no se pasa noise_model_json: coeficientes reales de
# aruco_pkg/noise_model.json (std = a + b*distancia, por eje de camara).
DEFAULT_NOISE_MODEL = {
    'tx': {'a': 0.0005491373304629619, 'b': 0.0004028992347095911},
    'ty': {'a': 0.0017686741562631886, 'b': 0.0008637123713288527},
    'tz': {'a': 0.0001, 'b': 0.007332070062677271},
}

CAM_OFFSET_X = -0.0596  # extrinseca camara->base_link de Parte A (graph_slam.py); ~6cm atras


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class VirtualArucoSensorNode(Node):
    def __init__(self):
        super().__init__('virtual_aruco_sensor_node')

        default_yaml = os.path.join(
            get_package_share_directory('aruco_sim_pkg'), 'worlds', 'aruco_landmarks_casa.yaml')

        self.declare_parameter('landmarks_yaml', default_yaml)
        self.declare_parameter('world_name', 'casa')  # 'casa' | 'casa_o'
        self.declare_parameter('robot_model_name', 'burger')
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('fov_deg', 80.0)
        self.declare_parameter('facing_angle_max_deg', 85.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('noise_model_json', '')
        self.declare_parameter('scale_uncertainty', 0.08)
        self.declare_parameter('bearing_floor_deg', 0.5)
        self.declare_parameter('log_csv_path', 'aruco_detections_sim.csv')
        self.declare_parameter('publish_viz', True)

        landmarks_yaml = self.get_parameter('landmarks_yaml').value
        self.world_name = self.get_parameter('world_name').value
        self.robot_model_name = self.get_parameter('robot_model_name').value
        self.max_range = float(self.get_parameter('max_range').value)
        self.fov_rad = math.radians(float(self.get_parameter('fov_deg').value))
        self.facing_max_rad = math.radians(float(self.get_parameter('facing_angle_max_deg').value))
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.scale_uncertainty = float(self.get_parameter('scale_uncertainty').value)
        self.bearing_floor = math.radians(float(self.get_parameter('bearing_floor_deg').value))
        self.log_csv_path = self.get_parameter('log_csv_path').value
        self.publish_viz = bool(self.get_parameter('publish_viz').value)

        noise_model_json = self.get_parameter('noise_model_json').value
        if noise_model_json:
            with open(noise_model_json, 'r') as f:
                self.noise_model = json.load(f)
        else:
            self.noise_model = DEFAULT_NOISE_MODEL

        self.landmarks = load_landmarks(landmarks_yaml)
        self.walls, self.furniture = get_obstacles(self.world_name)
        self.get_logger().info(
            f'{len(self.landmarks)} landmarks cargados de {landmarks_yaml} '
            f'(world={self.world_name}, {len(self.furniture)} muebles)')

        self._rng = random.Random(0)

        self.robot_pose = None  # (x, y, yaw)
        self.create_subscription(ModelStates, '/gazebo/model_states', self._model_states_cb, 10)

        self.det_pub = self.create_publisher(ArucoDetectionArray, '/aruco_detections', 10)
        if self.publish_viz:
            self.viz_pub = self.create_publisher(MarkerArray, '/aruco_landmarks_viz', 10)

        self._init_csv()

        period = 1.0 / max(publish_rate_hz, 1e-3)
        self.create_timer(period, self._tick)

    def _init_csv(self):
        is_new = not os.path.isfile(self.log_csv_path)
        self.csv_file = open(self.log_csv_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if is_new:
            self.csv_writer.writerow(
                ['timestamp', 'marker_id', 'tx', 'ty', 'tz', 'distancia_m', 'rx', 'ry', 'rz'])

    def _model_states_cb(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.robot_model_name)
        except ValueError:
            return
        pose = msg.pose[idx]
        yaw = yaw_from_quaternion(pose.orientation)
        self.robot_pose = (pose.position.x, pose.position.y, yaw)

    def _range_bearing_noise_std(self, rng):
        """Calcula el std de (rango, bearing) a una distancia dada.

        Propagado desde el modelo de ruido fiteado de Parte A (misma idea
        que graph_slam.py::_noise_obs_info, evaluada en bearing~0: ahi el
        Jacobiano r=hypot(bx,by) se reduce a std_range~=std_tz y
        std_bearing~=std_tx/rango).
        """
        tz = self.noise_model['tz']
        tx = self.noise_model['tx']
        std_tz = max(tz['a'] + tz['b'] * rng, 1e-4)
        std_tx = max(tx['a'] + tx['b'] * rng, 1e-4)
        std_range = math.hypot(std_tz, self.scale_uncertainty * rng)
        std_bearing = max(std_tx / max(rng, 1e-3), self.bearing_floor)
        return std_range, std_bearing

    def _tick(self):
        if self.robot_pose is None:
            return
        rx, ry, ryaw = self.robot_pose

        header_stamp = self.get_clock().now().to_msg()
        t = header_stamp.sec + header_stamp.nanosec * 1e-9

        det_array = ArucoDetectionArray()
        det_array.header.stamp = header_stamp
        det_array.header.frame_id = 'map'

        viz_markers = MarkerArray() if self.publish_viz else None

        for lm in self.landmarks.values():
            dx, dy = lm.x - rx, lm.y - ry
            rng = math.hypot(dx, dy)
            bearing = normalize_angle(math.atan2(dy, dx) - ryaw)

            visible = rng <= self.max_range and abs(bearing) <= self.fov_rad / 2.0
            if visible:
                # el marcador solo se ve si su cara (normal `lm.theta`) mira
                # hacia el robot, como un ArUco real pegado a una pared.
                angle_to_robot = math.atan2(ry - lm.y, rx - lm.x)
                facing_diff = normalize_angle(angle_to_robot - lm.theta)
                visible = abs(facing_diff) <= self.facing_max_rad
            if visible:
                visible = not is_occluded((rx, ry), (lm.x, lm.y), self.walls, self.furniture)

            if self.publish_viz:
                viz_markers.markers.append(self._make_viz_marker(lm, visible))

            if not visible:
                continue

            std_range, std_bearing = self._range_bearing_noise_std(rng)
            rng_noisy = max(rng + self._rng.gauss(0.0, std_range), 0.01)
            bearing_noisy = normalize_angle(bearing + self._rng.gauss(0.0, std_bearing))

            bx = rng_noisy * math.cos(bearing_noisy)
            by = rng_noisy * math.sin(bearing_noisy)
            # inversa de graph_slam.py::build_from_csv (bx=tz+cam_offset[0], by=-tx):
            tz = bx - CAM_OFFSET_X
            tx = -by
            ty = self._rng.gauss(0.0, self.noise_model['ty']['a'])

            det = ArucoDetection()
            det.marker_id = lm.id
            det.tx, det.ty, det.tz = tx, ty, tz
            det.range, det.bearing = rng_noisy, bearing_noisy
            det_array.detections.append(det)

            distancia_m = math.sqrt(tx * tx + ty * ty + tz * tz)
            self.csv_writer.writerow([t, lm.id, tx, ty, tz, distancia_m, 0.0, 0.0, 0.0])

        self.csv_file.flush()
        self.det_pub.publish(det_array)
        if self.publish_viz:
            self.viz_pub.publish(viz_markers)

    def _make_viz_marker(self, lm, visible):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'aruco_landmarks_viz'
        m.id = lm.id
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = lm.x
        m.pose.position.y = lm.y
        m.pose.position.z = 0.3
        m.pose.orientation.z = math.sin(lm.theta / 2.0)
        m.pose.orientation.w = math.cos(lm.theta / 2.0)
        m.scale.x, m.scale.y, m.scale.z = 0.02, 0.15, 0.15
        m.color.a = 1.0
        if visible:
            m.color.g = 1.0
        else:
            m.color.r = 1.0
        return m

    def destroy_node(self):
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VirtualArucoSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
