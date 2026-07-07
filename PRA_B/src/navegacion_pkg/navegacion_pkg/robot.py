#!/usr/bin/env python3
import collections
from scipy.signal import convolve2d
import heapq
import math
import numpy as np
import copy
from scipy.ndimage import binary_dilation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from enum import Enum, auto
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


class State(Enum):
    WAITING = auto()
    PLANNING = auto()
    WALKING = auto()
    AVOIDING = auto()
    ALIGNING = auto()


class RobotNavigator(Node):

    def __init__(self):
        super().__init__('robot_navigator')

        # --- Parámetro de robot ---
        self.declare_parameter('robot', 'tb3')
        self.declare_parameter('robot_id', 0)
        robot = self.get_parameter('robot').get_parameter_value().string_value
        robot_id = str(self.get_parameter('robot_id').get_parameter_value().integer_value)
        self.get_logger().info(f'Modo robot: {robot} (id={robot_id})')

        if robot == 'tb4':
            prefix = f'/tb4_{robot_id}'
            scan_topic = f'{prefix}/scan'
            cmd_topic = f'{prefix}/cmd_vel'
            self.offset_lidar_rad  = math.radians(90.0)  # LIDAR rotado 90° en TB4
            self.usar_intensidades = True  # descartar rayos con intensidad 0
        else:
            scan_topic = '/scan'
            cmd_topic = '/cmd_vel'
            self.offset_lidar_rad = 0.0
            self.usar_intensidades = False

        # Estado inicial
        self.state = State.WAITING
        self.get_logger().info(f'Estado inicial: {self.state.name}')

        # Variables internas
        self.map: OccupancyGrid | None = None
        self.current_pose: PoseStamped | None = None
        self.goal_pose: PoseStamped | None = None
        self.planned_path: list = []
        self.current_waypoint_idx: int = 0
        self.new_goal_received: bool = False
        self._avoid_state = 'FRENAR'
        self._avoid_yaw_start = None
        self.AVOID_ROTATION_ANGLE = math.radians(80.0)
        self.current_yaw = 0.0
        self._current_path_msg = None
        self._avoid_cooldown_until = 0.0

        # Detección de obstáculos
        self.last_scan: LaserScan | None = None
        self.obstacle_ahead: bool = False
        self.OBSTACLE_DISTANCE_THRESHOLD = 0.30   # margen para reaccionar ANTES de rozar (0.2 ≈ contacto)
        self.CONE_HALF_ANGLE = math.radians(30)   # cono frontal angosto: sólo lo realmente de frente
        self.min_front_dist = float('inf')        # última distancia mínima en el cono frontal (WALKING)

        # Evasión reactiva
        self.AVOID_ANGULAR_SPEED = 0.5
        self.AVOID_EVAL_HALF_ANGLE = math.radians(90)
        self.AVOID_CLEAR_MARGIN = 1.25
        self._avoid_turn_sign = 0.0

        # Inflado del mapa
        self.inflated_map: OccupancyGrid | None = None
        self.INFLATION_RADIUS_CELLS = 4

        # Pure Pursuit
        self.LOOKAHEAD_DISTANCE = 0.2
        self.LINEAR_SPEED = 0.1
        self.GOAL_TOLERANCE = 0.10
        self.ANGLE_TOLERANCE = math.radians(12.0)
        self.ANGULAR_SPEED = 0.3

        # Localización
        self.CONV_XY_THRESHOLD = 0.25
        self.CONV_THETA_THRESHOLD = 0.10
        self.DEGRADED_XY_THRESHOLD = 1.0
        self.DEGRADED_THETA_THRESHOLD = 0.30
        self.MIN_PARTICLES_FOR_STATS = 3
        self.belief_spread_xy: float | None = None
        self.belief_spread_theta: float | None = None
        self.localization_converged: bool = False

        # Ruido de Scan
        self.MAX_DYNAMIC_RANGE = 3.0 # corte de distancia (era hardcodeado)
        self.MIN_NEIGHBOR_HITS = 3 # filtro espacial
        self.PERSISTENCE_WINDOW = 5 # filtro temporal: memoria en scans
        self.PERSISTENCE_MIN = 3 # filtro temporal: apariciones exigidas
        self.DYNAMIC_INFLATION_RADIUS_CELLS = 4 # inflado del dinámico, separado del estático
        self._hit_history = collections.deque(maxlen=self.PERSISTENCE_WINDOW)
        # Capa dinámica persistente, actualizada continuamente (no sólo al planear).
        self.dynamic_cells: np.ndarray | None = None      # celdas dinámicas sin inflar
        self.dynamic_inflated: np.ndarray | None = None   # ya infladas: las consumen el planeo y la vigilancia del camino
        self._static_free_mask: np.ndarray | None = None  # True donde el mapa estático inflado está libre


        # Subscriptores
        qos_map = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.cb_map, qos_map)
        self.sub_belief = self.create_subscription(PoseArray, '/belief', self.cb_belief, 10)
        self.sub_pose = self.create_subscription(PoseStamped, '/estimated_pose', self.cb_pose, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.cb_goal, 10)
        self.sub_scan = self.create_subscription(LaserScan, scan_topic, self.cb_scan, 10)

        # Publicadores
        self.pub_cmd_vel = self.create_publisher(Twist, cmd_topic, 10)
        qos_latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_path = self.create_publisher(Path, '/planned_path', qos_latched)
        self.pub_lookahead = self.create_publisher(Marker, '/lookahead_point', 10)
        self.pub_planning_map = self.create_publisher(OccupancyGrid, '/planning_map', qos_latched)

        self.timer = self.create_timer(0.1, self.state_machine_loop)


    # CALLBACKS

    def cb_map(self, msg: OccupancyGrid):
        """
        Message callback for receiving the occupancy grid map. It inflates the map to account for the robot's size and obstacles.
        """
        # El map_publisher reenvía el MISMO mapa cada ~1s. Si no cambió, evitamos
        # re-inflar (dilatación cara) y no ensuciamos el log.
        if (self.map is not None
                and msg.info.width == self.map.info.width
                and msg.info.height == self.map.info.height
                and msg.data == self.map.data):
            return

        self.map = msg
        self.inflated_map = self._inflate_map(msg)
        # Máscara de espacio libre en el mapa estático inflado. La capa dinámica
        # sólo considera hits que caen aquí: así los rayos que golpean paredes ya
        # mapeadas NO se cuentan como obstáculos dinámicos (evita engrosar las
        # paredes y bloquear la ruta con falsos positivos).
        info = self.inflated_map.info
        inflated = np.array(self.inflated_map.data, dtype=np.int8).reshape((info.height, info.width))
        self._static_free_mask = (inflated == 0)
        self.get_logger().info('Mapa recibido e inflado.')

    def cb_pose(self, msg: PoseStamped):
        """
        Message callback for receiving the robot's estimated pose. It extracts the yaw angle from the quaternion orientation.
        """
        self.current_pose = msg
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def cb_goal(self, msg: PoseStamped):
        """
        Message callback for receiving a new goal pose. It sets the goal and flags that a new goal has been received.
        """
        self.get_logger().info(f'Nuevo goal recibido: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self.goal_pose = msg
        self.new_goal_received = True

    def cb_scan(self, msg: LaserScan):
        """
        Message callback for receiving LIDAR scan data. It processes the scan to detect obstacles in front of the robot and logs relevant information.
        """
        self.last_scan = msg

        # Debug metadata (una sola vez)
        if not hasattr(self, '_scan_debug_printed'):
            self.get_logger().info(
                f'[SCAN META] frame_id={msg.header.frame_id} '
                f'angle_min={math.degrees(msg.angle_min):.1f}° '
                f'angle_max={math.degrees(msg.angle_max):.1f}° '
                f'incr={math.degrees(msg.angle_increment):.2f}° '
                f'n_rays={len(msg.ranges)} '
                f'offset_lidar={math.degrees(self.offset_lidar_rad):.1f}°'
            )
            self._scan_debug_printed = True

        # El aspecto dinámico del mapa se actualiza SIEMPRE (todos los estados),
        # no sólo al planear: así un replaneo ve lo último del LIDAR y el filtro
        # temporal de persistencia acumula una ventana real de scans consecutivos.
        self._update_dynamic_obstacles()
        # Publicar el mapa de planeo (estático + dinámico) en vivo para RViz.
        self._publish_planning_map()

        # Solo detecto obstáculos en estado WALKING
        if self.state not in (State.WALKING,):
            return

        angle = msg.angle_min
        obstacle_found = False
        min_front_dist = float('inf')

        for i, r in enumerate(msg.ranges):
            # Filtro de intensidades (TB4: descartar lecturas con intensidad 0)
            if self.usar_intensidades and msg.intensities and msg.intensities[i] == 0.0:
                angle += msg.angle_increment
                continue

            # Ángulo del rayo en frame robot (LIDAR frame + offset)
            robot_angle = angle + self.offset_lidar_rad
            angle_norm = math.atan2(math.sin(robot_angle), math.cos(robot_angle))

            if abs(angle_norm) <= self.CONE_HALF_ANGLE:
                if r > msg.range_min and r < msg.range_max and math.isfinite(r):
                    # Sin break: recorremos todo el cono para quedarnos con el
                    # mínimo real (lo necesita el chequeo de emergencia).
                    min_front_dist = min(min_front_dist, r)
                    if r < self.OBSTACLE_DISTANCE_THRESHOLD:
                        obstacle_found = True

            angle += msg.angle_increment

        self.get_logger().info(
            f'[SCAN] min_front={min_front_dist:.2f}m  threshold={self.OBSTACLE_DISTANCE_THRESHOLD}m  found={obstacle_found}',
            throttle_duration_sec=3.0)

        self.min_front_dist = min_front_dist
        self.obstacle_ahead = obstacle_found

    def cb_belief(self, msg: PoseArray):
        """
        Message callback for receiving the belief (particle filter) as a PoseArray. It calculates the spread of the belief in both position and orientation.
        """
        n = len(msg.poses)
        if n < self.MIN_PARTICLES_FOR_STATS:
            self.belief_spread_xy = None
            self.belief_spread_theta = None
            return

        xs = np.array([p.position.x for p in msg.poses])
        ys = np.array([p.position.y for p in msg.poses])
        self.belief_spread_xy = float(xs.var() + ys.var())

        yaws = np.array([self._get_yaw_from_pose_msg(p) for p in msg.poses])
        cos_mean = np.cos(yaws).mean()
        sin_mean = np.sin(yaws).mean()
        R = math.hypot(cos_mean, sin_mean)
        self.belief_spread_theta = float(1.0 - R)


    # LOOP PRINCIPAL DE LA MÁQUINA DE ESTADOS

    def state_machine_loop(self):
        """
        Main state machine loop.
        """
        if not self._localization_ok():
            self.get_logger().warn(
                f'Localización no confiable. Robot detenido. '
                f'Estado en espera: {self.state.name}.',
                throttle_duration_sec=2.0)
            return

        match self.state:
            case State.WAITING: self.run_waiting()
            case State.PLANNING: self.run_planning()
            case State.WALKING: self.run_walking()
            case State.AVOIDING: self.run_avoiding()
            case State.ALIGNING: self.run_aligning()


    # IMPLEMENTACIÓN DE CADA ESTADO

    def _localization_ok(self) -> bool:
        """
        Check if the localization is ok.
        """
        return (self._check_localization_converged("localization_ok")
                and not self._localization_degraded())

    def run_waiting(self):
        """
        Run the WAITING state. If a new goal is received, transition to PLANNING.
        """
        if self.new_goal_received:
            self.new_goal_received = False
            self.get_logger().info('Goal recibido. Pasando a PLANNING.')
            self.transition_to(State.PLANNING)

    def run_planning(self):
        """
        Run the PLANNING state. It builds the planning map, runs the path planning algorithm, and transitions to WALKING if a path is found.
        """
        if self.inflated_map is None or self.current_pose is None or self.goal_pose is None:
            self.get_logger().warn('Faltan datos para planificar.')
            return
        
        planning_map = self._publish_planning_map()

        path = self._run_theta_star(self.current_pose, self.goal_pose, planning_map, self.map)

        if path:
            self.planned_path = path
            self.current_waypoint_idx = 0
            self.get_logger().info(f'Camino encontrado ({len(path)} waypoints). Pasando a WALKING.')
            self._publish_path(path)
            self.transition_to(State.WALKING)
        else:
            self.get_logger().warn('No se encontró camino. Volviendo a WAITING.')
            self.transition_to(State.WAITING)

    def run_walking(self):
        """
        Run the WALKING state. It checks for obstacles, goal reach, and computes the control command using Pure Pursuit.
        """
        if self._current_path_msg:
            self._current_path_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_path.publish(self._current_path_msg)

        if self.new_goal_received:
            self.new_goal_received = False
            self.get_logger().info('Nuevo goal durante WALKING. Pasando a PLANNING.')
            self._stop_robot()
            self.transition_to(State.PLANNING)
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now >= self._avoid_cooldown_until and self._obstacle_detected_on_path():
            self.get_logger().info('Obstáculo al frente. Pasando a AVOIDING.')
            self._stop_robot()
            self.transition_to(State.AVOIDING)
            return

        # Replaneo proactivo: si un obstáculo dinámico ya cayó sobre la ruta
        # (capa mantenida en tiempo real), replaneamos antes de chocar de frente.
        if now >= self._avoid_cooldown_until and self._planned_path_blocked():
            self.get_logger().info('Camino bloqueado por obstáculo dinámico. Re-planeando.')
            self._stop_robot()
            self._avoid_cooldown_until = now + 1.0
            self.transition_to(State.PLANNING)
            return

        if self._reached_goal_position():
            self.get_logger().info('Posición del goal alcanzada. Pasando a ALIGNING.')
            self._stop_robot()
            self.transition_to(State.ALIGNING)
            return

        cmd = self._compute_pure_pursuit_cmd()
        self.pub_cmd_vel.publish(cmd)

    def run_avoiding(self):
        """
        Run the AVOIDING state. It executes the avoidance maneuver and checks for convergence of the particle filter before transitioning back to PLANNING.
        """
        avoidance_done = self._execute_avoidance_maneuver()

        if avoidance_done:
            self._stop_robot()
            if (self.belief_spread_xy is not None and
                    self.belief_spread_xy > self.CONV_XY_THRESHOLD * 3):
                self.get_logger().info(
                    f'Post-AVOIDING: esperando reconvergencia PF '
                    f'(spread_xy={self.belief_spread_xy:.3f})',
                    throttle_duration_sec=0.5)
                return
            now = self.get_clock().now().nanoseconds / 1e9
            self._avoid_cooldown_until = now + 1.0
            self.get_logger().info('Evasión completa + PF reconvergido. Re-planeando.')
            self.transition_to(State.PLANNING)

    def run_aligning(self):
        """
        Run the ALIGNING state. It aligns the robot to the goal orientation and transitions to WAITING or PLANNING based on whether a new goal was received.
        """
        alignment_done = self._align_to_goal_angle()
        self.get_logger().info(f"[ALIGNING STATE] en aligning: {self.new_goal_received}")

        if alignment_done:
            self._stop_robot()
            if self.new_goal_received:
                self.new_goal_received = False
                self.get_logger().info('Nuevo goal durante ALIGNING. Pasando a PLANNING.')
                self.transition_to(State.PLANNING)
            else:
                self.get_logger().info('Alineación completa. Pasando a WAITING.')
                self.transition_to(State.WAITING)

    def transition_to(self, new_state: State):
        """
        Transition to a new state and log the transition.
        """
        self.get_logger().info(f'[FSM] {self.state.name} → {new_state.name}')
        self.state = new_state


    # HELPERS

    def _check_localization_converged(self, msg: str) -> bool:
        """
        Checks if the localization has converged based on the belief spread in position and orientation.
        """
        if self.belief_spread_xy is None or self.belief_spread_theta is None:
            return True
        if self.localization_converged:
            return True
        if (self.belief_spread_xy < self.CONV_XY_THRESHOLD
                and self.belief_spread_theta < self.CONV_THETA_THRESHOLD):
            self.localization_converged = True
            self.get_logger().info(
                f'Localización convergió en {msg}'
                f'(spread_xy={self.belief_spread_xy:.3f} m², '
                f'spread_θ={self.belief_spread_theta:.3f}).')
            return True
        return False
    
    def _update_dynamic_obstacles(self):
        """
        Fold the latest LIDAR scan into a persistent grid of dynamic obstacles.
        Runs on EVERY scan (all states), not only while planning, so a replan
        always sees the freshest obstacles and the temporal filter accumulates a
        real window of consecutive scans.

        Two chained noise filters:
          1) SPATIAL (3x3 neighborhood count): a cell with hits survives only if
             it has >= MIN_NEIGHBOR_HITS occupied neighbors -> kills isolated
             rays within a single scan.
          2) TEMPORAL (persistence): a spatially-accepted cell is kept only if it
             appears in >= PERSISTENCE_MIN of the last PERSISTENCE_WINDOW scans
             -> kills flickering cells over time.

        The surviving cells are inflated once and cached in self.dynamic_inflated
        so that _build_planning_map() and _planned_path_blocked() consume them
        cheaply (no re-projection, no re-dilation per call).
        """
        if self.inflated_map is None or self.current_pose is None or self.last_scan is None:
            return

        info = self.inflated_map.info
        width, height = info.width, info.height

        # Pose del robot en el mundo
        rx = self.current_pose.pose.position.x
        ry = self.current_pose.pose.position.y
        ryaw = self._get_yaw_from_pose(self.current_pose)

        scan = self.last_scan
        angle = scan.angle_min

        # --- Acumular hits crudos de este scan en una grilla de CONTEO ---
        # Guardamos CUÁNTOS hits caen en cada celda (no un booleano), porque
        # el filtro espacial necesita densidad, no sólo presencia.
        raw_counts = np.zeros((height, width), dtype=np.int32)

        for i, rng in enumerate(scan.ranges):
            a = angle
            angle += scan.angle_increment

            # Filtro de intensidades (TB4: descartar lecturas con intensidad 0)
            if self.usar_intensidades and scan.intensities and scan.intensities[i] == 0.0:
                continue
            # Descartar lecturas inválidas o fuera de rango útil
            if not math.isfinite(rng):
                continue
            if rng < scan.range_min or rng > scan.range_max:
                continue
            # Sólo nos importan obstáculos razonablemente cerca
            if rng > self.MAX_DYNAMIC_RANGE:
                continue

            # Punto de impacto en marco del robot -> marco mundo.
            # Se suma el offset del LIDAR (rplidar del TB4 rotado +90°), igual
            # que en el resto de las proyecciones. Sin esto, los hits salen
            # rotados 90° y aparecen como "obstáculos" en espacio abierto.
            wx = rx + rng * math.cos(ryaw + a + self.offset_lidar_rad)
            wy = ry + rng * math.sin(ryaw + a + self.offset_lidar_rad)

            # Inline (sin _world_to_grid) para no spamear warnings a la tasa del LIDAR.
            col = int((wx - info.origin.position.x) / info.resolution)
            row = int((wy - info.origin.position.y) / info.resolution)
            if 0 <= row < height and 0 <= col < width:
                raw_counts[row, col] += 1

        # --- MÁSCARA ESTÁTICA: descartar hits sobre paredes ya mapeadas ---
        # Un rayo que golpea una pared conocida NO es un obstáculo dinámico. Sin
        # esto, las paredes (persistentes en el scan) llenan la capa dinámica y,
        # tras inflar, bloquean la ruta con falsos positivos.
        if self._static_free_mask is not None:
            raw_counts[~self._static_free_mask] = 0

        # --- FILTRO 1: ESPACIAL (conteo por vecindad 3x3) ---
        # Sumamos los hits de cada celda con los de sus 8 vecinas. Una celda
        # sobrevive si esa suma alcanza MIN_NEIGHBOR_HITS.
        kernel3 = np.ones((3, 3), dtype=np.int32)
        neighbor_sum = convolve2d(raw_counts, kernel3, mode='same')
        spatial_cells = (raw_counts > 0) & (neighbor_sum >= self.MIN_NEIGHBOR_HITS)

        # --- FILTRO 2: TEMPORAL (persistencia sobre la ventana de scans) ---
        # Ahora que esto corre en cada scan, el historial acumula scans
        # consecutivos reales y el filtro de persistencia sí tiene sentido.
        self._hit_history.append(spatial_cells)
        appearances = np.zeros((height, width), dtype=np.int32)
        for past in self._hit_history:
            appearances += past.astype(np.int32)
        dynamic_cells = appearances >= self.PERSISTENCE_MIN

        # Inflar UNA sola vez con el radio dedicado al dinámico y cachear.
        r = self.DYNAMIC_INFLATION_RADIUS_CELLS
        y_k, x_k = np.ogrid[-r:r+1, -r:r+1]
        kernel = (x_k**2 + y_k**2) <= r**2
        self.dynamic_cells = dynamic_cells
        self.dynamic_inflated = binary_dilation(dynamic_cells, structure=kernel)

        self.get_logger().info(
            f'Dinámico: {int((raw_counts > 0).sum())} celdas crudas -> '
            f'{int(spatial_cells.sum())} espacial -> '
            f'{int(dynamic_cells.sum())} persistente (inflado r={r}).',
            throttle_duration_sec=3.0)

    def _build_planning_map(self) -> OccupancyGrid:
        """
        Static inflated map + the latest cached dynamic obstacles
        (self.dynamic_inflated, maintained continuously by
        _update_dynamic_obstacles). This function no longer touches the LIDAR;
        it just merges the current dynamic layer onto a fresh copy of the static
        map. A cell is blocked only if it survived both the spatial and temporal
        filters.
        """
        # Partimos siempre del mapa estático inflado (nunca se muta).
        planning = copy.deepcopy(self.inflated_map)

        if self.dynamic_inflated is None:
            return planning

        info = planning.info
        width, height = info.width, info.height
        data = np.array(planning.data, dtype=np.int8).reshape((height, width))

        # Marcar como ocupado sólo lo que hoy es libre (no tocar desconocido).
        data[self.dynamic_inflated & (data == 0)] = 100
        planning.data = data.flatten().tolist()
        return planning

    def _publish_planning_map(self) -> OccupancyGrid | None:
        """
        Build the current planning map (static + live dynamic layer), publish it
        to /planning_map for RViz, and return it. Returns None if the static map
        is not available yet. Called both continuously (from cb_scan) and right
        before planning.
        """
        if self.inflated_map is None:
            return None

        planning_map = self._build_planning_map()
        planning_map.header.stamp = self.get_clock().now().to_msg()
        if not planning_map.header.frame_id:
            planning_map.header.frame_id = 'map'
        self.pub_planning_map.publish(planning_map)
        return planning_map

    def _planned_path_blocked(self) -> bool:
        """
        Check whether some waypoint AHEAD on the current path is now covered by a
        NEW dynamic obstacle, so the robot can replan proactively.

        Two guards avoid spurious replans:
          - A waypoint counts only if it is a dynamic obstacle on a cell the
            STATIC map considers free. Mapped walls are persistent in the scan,
            so comparing against the raw dynamic layer would flag almost every
            path (paths hug the inflated walls); requiring static-free isolates
            genuinely new obstacles.
          - Waypoints within the robot's near field are skipped. There are always
            returns there (walls the robot legitimately passes, or an obstacle it
            is already leaving behind); the reactive AVOIDING state covers the
            immediate front. Only what appears further along the route matters.
        """
        if (self.dynamic_inflated is None or not self.planned_path
                or self.inflated_map is None or self.current_pose is None):
            return False

        info = self.inflated_map.info
        rx = self.current_pose.pose.position.x
        ry = self.current_pose.pose.position.y
        near_skip = self.DYNAMIC_INFLATION_RADIUS_CELLS * info.resolution + self.LOOKAHEAD_DISTANCE

        for wp in self.planned_path[self.current_waypoint_idx:]:
            wx = wp.pose.position.x
            wy = wp.pose.position.y
            if math.hypot(wx - rx, wy - ry) < near_skip:
                continue
            col = int((wx - info.origin.position.x) / info.resolution)
            row = int((wy - info.origin.position.y) / info.resolution)
            if not (0 <= row < info.height and 0 <= col < info.width):
                continue
            static_val = self.inflated_map.data[row * info.width + col]
            if self.dynamic_inflated[row, col] and static_val == 0:
                return True
        return False

    def _line_of_sight(self, grid: OccupancyGrid, r0: int, c0: int, r1: int, c1: int) -> bool:
        """
        Bresenham's line algorithm to check if there is a clear line of sight between two grid cells.
        """
        width = grid.info.width
        dr = abs(r1 - r0); dc = abs(c1 - c0)
        sr = 1 if r1 > r0 else -1
        sc = 1 if c1 > c0 else -1
        r, c = r0, c0
        err = dr - dc
        while True:
            idx = r * width + c
            val = grid.data[idx]
            if val >= 50 or val == -1:
                return False
            if r == r1 and c == c1:
                return True
            e2 = 2 * err
            if e2 > -dc:
                err -= dc; r += sr
            if e2 < dr:
                err += dr; c += sc

    def _run_theta_star(self, start: PoseStamped, goal: PoseStamped, inflated_grid: OccupancyGrid, grid: OccupancyGrid) -> list:
        """
        Run the Theta* path planning algorithm on the inflated grid. Returns a list of waypoints in world coordinates.
        """
        sx = start.pose.position.x; sy = start.pose.position.y
        gx = goal.pose.position.x; gy = goal.pose.position.y

        start_cell = self._world_to_grid(sx, sy, grid)
        goal_cell = self._world_to_grid(gx, gy, grid)

        if start_cell is None or goal_cell is None:
            self.get_logger().error('Theta*: start o goal fuera del mapa.')
            return []

        rs, cs = start_cell
        rg, cg = goal_cell
        width = grid.info.width

        if grid.data[rs * width + cs] >= 50:
            self.get_logger().error('Theta*: celda de inicio ocupada.')
            return []
        if grid.data[rg * width + cg] >= 50:
            self.get_logger().error('Theta*: celda de goal ocupada.')
            return []

        def h(r, c):
            """
            Heuristic function for A* (Euclidean distance to goal).
            """
            return math.hypot(r - rg, c - cg)

        g_score = {(rs, cs): 0.0}
        parent = {(rs, cs): (rs, cs)}
        open_set = []
        heapq.heappush(open_set, (h(rs, cs), rs, cs))
        closed_set = set()

        neighbors_offsets = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),  (1, 1, math.sqrt(2)),
        ]
        height = inflated_grid.info.height

        while open_set:
            _, r, c = heapq.heappop(open_set)
            current = (r, c)
            if current in closed_set:
                continue
            closed_set.add(current)
            if current == (rg, cg):
                break
            pr, pc = parent[current]
            for dr, dc, move_cost in neighbors_offsets:
                nr, nc = r + dr, c + dc
                neighbor = (nr, nc)
                if not (0 <= nr < height and 0 <= nc < width):
                    continue
                if neighbor in closed_set:
                    continue
                cell_val = inflated_grid.data[nr * width + nc]
                if cell_val >= 50 or cell_val == -1:
                    continue
                if self._line_of_sight(inflated_grid, pr, pc, nr, nc):
                    g_via_grandparent = g_score[(pr, pc)] + math.hypot(nr - pr, nc - pc)
                    if g_via_grandparent < g_score.get(neighbor, float('inf')):
                        g_score[neighbor] = g_via_grandparent
                        parent[neighbor] = (pr, pc)
                        f = g_via_grandparent + h(nr, nc)
                        heapq.heappush(open_set, (f, nr, nc))
                else:
                    g_via_current = g_score[current] + move_cost
                    if g_via_current < g_score.get(neighbor, float('inf')):
                        g_score[neighbor] = g_via_current
                        parent[neighbor] = current
                        f = g_via_current + h(nr, nc)
                        heapq.heappush(open_set, (f, nr, nc))

        if (rg, cg) not in parent:
            self.get_logger().warn('Theta*: no se encontró camino.')
            return []

        path_cells = []
        node = (rg, cg)
        while node != (rs, cs):
            path_cells.append(node)
            node = parent[node]
        path_cells.append((rs, cs))
        path_cells.reverse()

        waypoints = []
        for (row, col) in path_cells:
            wx, wy = self._grid_to_world(row, col, inflated_grid)
            waypoints.append(self._make_pose_stamped(wx, wy))

        self.get_logger().info(f'Theta*: camino encontrado con {len(waypoints)} waypoints.')
        return waypoints

    def _world_to_grid(self, x: float, y: float, grid: OccupancyGrid) -> tuple[int, int] | None:
        """
        Convert world coordinates (x, y) to grid cell indices (row, col).
        """
        info = grid.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        if 0 <= row < info.height and 0 <= col < info.width:
            return (row, col)
        self.get_logger().warn(f'_world_to_grid: ((!) {x:.2f}, {y:.2f}) fuera del mapa.')
        return None

    def _grid_to_world(self, row: int, col: int, grid: OccupancyGrid) -> tuple[float, float]:
        """
        Convert grid cell indices (row, col) to world coordinates (x, y).
        """
        info = grid.info
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return (x, y)

    def _make_pose_stamped(self, x: float, y: float) -> PoseStamped:
        """
        Create a PoseStamped message with the given world coordinates (x, y) and a default orientation.
        """
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        return ps

    def _publish_path(self, waypoints: list):
        """
        Publish the planned path as a Path message for visualization in RViz.
        """
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = waypoints
        self._current_path_msg = path_msg
        self.pub_path.publish(path_msg)

    def _obstacle_detected_on_path(self) -> bool:
        """
        Trigger reactive avoidance for ANY obstacle physically close in the front
        cone, whether or not it is in the map. The map is only for PLANNING; the
        reactive layer exists so the robot never drives into something it can see
        — including mapped walls it drifts toward. Not reacting here is what let
        it graze walls, and the collision then corrupted localization.

        (Whether an obstacle is *new* still matters, but only for the proactive
        replan via the dynamic layer — see _planned_path_blocked.)
        """
        return self.obstacle_ahead

    def _obstacle_is_unmapped(self) -> bool:
        """
        Check if the detected obstacle is not present in the inflated map. 
        This is done by projecting the LIDAR scan points into the map and checking if they fall into free space or occupied cells.
        """
        if self.last_scan is None or self.inflated_map is None or self.current_pose is None:
            return False

        msg = self.last_scan
        rx = self.current_pose.pose.position.x
        ry = self.current_pose.pose.position.y
        robot_yaw = self.current_yaw
        info = self.inflated_map.info

        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            # Filtro de intensidades (TB4)
            if self.usar_intensidades and msg.intensities and msg.intensities[i] == 0.0:
                angle += msg.angle_increment
                continue

            robot_angle = angle + self.offset_lidar_rad
            angle_norm = math.atan2(math.sin(robot_angle), math.cos(robot_angle))

            if abs(angle_norm) <= self.CONE_HALF_ANGLE:
                if (r > msg.range_min and r < msg.range_max
                        and math.isfinite(r)
                        and r < self.OBSTACLE_DISTANCE_THRESHOLD):
                    global_angle = robot_yaw + angle_norm
                    hit_x = rx + r * math.cos(global_angle)
                    hit_y = ry + r * math.sin(global_angle)

                    col = int((hit_x - info.origin.position.x) / info.resolution)
                    row = int((hit_y - info.origin.position.y) / info.resolution)

                    if 0 <= row < info.height and 0 <= col < info.width:
                        cell_val = self.inflated_map.data[row * info.width + col]
                        if cell_val < 50:
                            return True

            angle += msg.angle_increment
        return False

    def _reached_goal_position(self) -> bool:
        """
        Check if the robot has reached the goal position within a certain tolerance.
        """
        if self.current_pose is None or self.goal_pose is None:
            return False
        dx = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
        return math.hypot(dx, dy) < self.GOAL_TOLERANCE

    def _get_yaw_from_pose(self, pose: PoseStamped) -> float:
        """
        Extract the yaw angle from a PoseStamped message.
        """
        q = pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _get_yaw_from_pose_msg(self, pose) -> float:
        """
        Extract the yaw angle from a Pose message (not PoseStamped).
        """
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _compute_pure_pursuit_cmd(self) -> Twist:
        """
        Compute the control command using the Pure Pursuit algorithm based on the current pose and the planned path.
        """
        if self.current_pose is None or not self.planned_path:
            return Twist()

        rx = self.current_pose.pose.position.x
        ry = self.current_pose.pose.position.y
        robot_yaw = self._get_yaw_from_pose(self.current_pose)

        while self.current_waypoint_idx < len(self.planned_path) - 1:
            wp = self.planned_path[self.current_waypoint_idx]
            dx = wp.pose.position.x - rx
            dy = wp.pose.position.y - ry
            if math.hypot(dx, dy) < self.LOOKAHEAD_DISTANCE:
                self.current_waypoint_idx += 1
            else:
                break

        if self.current_waypoint_idx < len(self.planned_path):
            target_wp = self.planned_path[self.current_waypoint_idx]
        else:
            target_wp = self.planned_path[-1]

        dx = target_wp.pose.position.x - rx
        dy = target_wp.pose.position.y - ry
        angle_to_target = math.atan2(dy, dx)
        alpha = math.atan2(
            math.sin(angle_to_target - robot_yaw),
            math.cos(angle_to_target - robot_yaw)
        )

        L = self.LOOKAHEAD_DISTANCE
        v = self.LINEAR_SPEED
        omega = (2.0 * v * math.sin(alpha)) / L

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega

        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'pp'; m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = target_wp.pose.position.x
        m.pose.position.y = target_wp.pose.position.y
        m.pose.position.z = 0.1
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
        self.pub_lookahead.publish(m)

        return cmd

    def _execute_avoidance_maneuver(self) -> bool:
        """
        Execute the avoidance maneuver by rotating the robot in place until it has turned a specified angle away from the obstacle.
        """
        if self.last_scan is None:
            self._stop_robot()
            return False

        if self._avoid_yaw_start is None:
            self._avoid_yaw_start = self._get_yaw_from_pose(self.current_pose)
            self._avoid_turn_sign = self._pick_clearer_side()
            lado = 'izquierda' if self._avoid_turn_sign > 0 else 'derecha'
            self.get_logger().info(f'Obstáculo: girando {self.AVOID_ROTATION_ANGLE}° hacia la {lado}.')

        yaw_now = self._get_yaw_from_pose(self.current_pose)
        delta = abs(math.atan2(math.sin(yaw_now - self._avoid_yaw_start), math.cos(yaw_now - self._avoid_yaw_start)))

        if delta >= self.AVOID_ROTATION_ANGLE:
            self._stop_robot()
            self._avoid_turn_sign = 0.0
            self._avoid_yaw_start = None
            self.get_logger().info('Rotación de evasión completa.')
            return True

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self._avoid_turn_sign * self.AVOID_ANGULAR_SPEED
        self.pub_cmd_vel.publish(cmd)
        return False

    def _pick_clearer_side(self) -> float:
        """
        Determine which side (left or right) has more free space based on the last LIDAR scan.
        """
        msg = self.last_scan
        min_left = float('inf')
        min_right = float('inf')

        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if self.usar_intensidades and msg.intensities and msg.intensities[i] == 0.0:
                angle += msg.angle_increment
                continue

            robot_angle = angle + self.offset_lidar_rad
            angle_norm = math.atan2(math.sin(robot_angle), math.cos(robot_angle))

            if abs(angle_norm) <= self.AVOID_EVAL_HALF_ANGLE:
                if r > msg.range_min and r < msg.range_max and math.isfinite(r):
                    if angle_norm > 0:
                        min_left = min(min_left, r)
                    elif angle_norm < 0:
                        min_right = min(min_right, r)

            angle += msg.angle_increment

        return +1.0 if min_left >= min_right else -1.0

    def _align_to_goal_angle(self) -> bool:
        """
        Align the robot to the goal orientation using a simple proportional controller. 
        Returns True if alignment is complete.
        """
        if self.current_pose is None or self.goal_pose is None:
            return False
        if self.new_goal_received:
            return True

        current_yaw = self._get_yaw_from_pose(self.current_pose)
        goal_yaw = self._get_yaw_from_pose(self.goal_pose)
        error = math.atan2(math.sin(goal_yaw - current_yaw), math.cos(goal_yaw - current_yaw))

        if abs(error) < self.ANGLE_TOLERANCE:
            self._stop_robot()
            self.get_logger().info(f'Alineación completa. Error residual: {math.degrees(error):.1f}°')
            return True

        cmd = Twist()
        cmd.angular.z = math.copysign(self.ANGULAR_SPEED, error)
        self.pub_cmd_vel.publish(cmd)
        return False

    def _localization_degraded(self) -> bool:
        """
        Check if the localization is degraded based on the belief spread in position and orientation.
        """
        if self.belief_spread_xy is None or self.belief_spread_theta is None:
            return False
        degraded = (self.belief_spread_xy > self.DEGRADED_XY_THRESHOLD
                    or self.belief_spread_theta > self.DEGRADED_THETA_THRESHOLD)
        if degraded and self.localization_converged:
            self.localization_converged = False
            self.get_logger().warn(
                f'Localización DEGRADADA '
                f'(spread_xy={self.belief_spread_xy:.3f} m², '
                f'spread_θ={self.belief_spread_theta:.3f}). '
                f'Robot detenido hasta re-converger.')
        return degraded

    def _inflate_map(self, grid: OccupancyGrid) -> OccupancyGrid:
        """
        Inflate the occupied cells in the occupancy grid by a specified radius to account for the robot's size and safety margin.
        """
        r = self.INFLATION_RADIUS_CELLS
        width = grid.info.width
        height = grid.info.height
        raw = np.array(grid.data, dtype=np.int8).reshape((height, width))
        occupied = (raw == 100)
        y_k, x_k = np.ogrid[-r:r+1, -r:r+1]
        kernel = (x_k**2 + y_k**2) <= r**2
        dilated = binary_dilation(occupied, structure=kernel)
        inflated_raw = raw.copy()
        inflate_mask = dilated & (raw == 0)
        inflated_raw[inflate_mask] = 100
        inflated_grid = copy.deepcopy(grid)
        inflated_grid.data = inflated_raw.flatten().tolist()
        self.get_logger().info(
            f'Mapa inflado: {int(inflate_mask.sum())} celdas nuevas bloqueadas '
            f'(radio={r} celdas = {r * grid.info.resolution:.2f} m).'
        )
        return inflated_grid

    def _stop_robot(self):
        """
        Stop the robot by publishing a zero Twist command.
        """
        self.pub_cmd_vel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()