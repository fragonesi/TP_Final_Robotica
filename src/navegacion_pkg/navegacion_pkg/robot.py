#!/usr/bin/env python3
"""
Nodo de navegación autónoma - Esqueleto con máquina de estados.
Estados: WAITING → PLANNING → WALKING → AVOIDING → ALIGNING → WAITING
"""
import heapq
import math
import numpy as np
import copy
from scipy.ndimage import binary_dilation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from enum import Enum, auto

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


# ---------------------------------------------------------------------------
# Definición de estados
# ---------------------------------------------------------------------------

class State(Enum):
    WAITING    = auto()
    PLANNING   = auto()
    WALKING    = auto()
    AVOIDING   = auto()
    ALIGNING   = auto()
    # RELOCALIZING = auto()


# ---------------------------------------------------------------------------
# Nodo principal
# ---------------------------------------------------------------------------

class RobotNavigator(Node):

    def __init__(self):
        super().__init__('robot_navigator')

        # Estado inicial ---------
        self.state = State.WAITING
        self.get_logger().info(f'Estado inicial: {self.state.name}')

        # Variables internas ---------
        self.map: OccupancyGrid | None = None
        self.current_pose: PoseStamped | None = None
        self.goal_pose: PoseStamped | None = None
        self.planned_path: list = [] # lista de waypoints
        self.current_waypoint_idx: int = 0
        self.new_goal_received: bool = False # se usa para goal nuevo durante WALKING
        self._avoid_state = 'FRENAR'
        self._avoid_yaw_start = None
        self.AVOID_ROTATION_ANGLE = math.radians(80.0)
        self.current_yaw = 0.0
        self._current_path_msg = None
        self._avoid_cooldown_until = 0.0  # timestamp en segundos

        # --- Detección de obstáculos ---
        self.last_scan: LaserScan | None = None
        self.obstacle_ahead: bool = False # flag de si hay un obstaculo delante
        self.OBSTACLE_DISTANCE_THRESHOLD = 0.2 # en metros 
        self.CONE_HALF_ANGLE = math.radians(45) # ±45°: un cono delantero de un total de 90°

        # --- Evasión reactiva de obstáculos ---
        self.AVOID_ANGULAR_SPEED = 0.5 # rad/s — velocidad de giro al esquivar
        self.AVOID_EVAL_HALF_ANGLE = math.radians(90)  # +-90°: sector que se mira para elegir lado
        self.AVOID_CLEAR_MARGIN = 1.25  # el frente se da por libre si la dist. mínima supera OBSTACLE_DISTANCE_THRESHOLD x este margen
        self._avoid_turn_sign = 0.0 # +1 izquierda / -1 derecha

        # --- Inflar el mapa ---
        self.inflated_map: OccupancyGrid | None = None
        self.INFLATION_RADIUS_CELLS = 4  # 3 celdas × 0.05 m = 0.15 m de margen

        # --- Pure Pursuit ---
        self.LOOKAHEAD_DISTANCE = 0.2   # metros
        self.LINEAR_SPEED       = 0.1  # m/s — constante
        self.GOAL_TOLERANCE     = 0.10  # metros — distancia para considerar que llegó
        self.ANGLE_TOLERANCE    = math.radians(12.0)  # ±5° para considerar alineado
        self.ANGULAR_SPEED      = 0.3   # rad/s — velocidad de giro en ALIGNING

        # --- Watchdog de progreso en WALKING ---
        # Si el robot queda trabado (choque, patinaje) sin acercarse al goal,
        # replanifica en vez de caminar indefinidamente sin avanzar.
        self.WALKING_STUCK_TIMEOUT = 8.0  # segundos sin progreso antes de replanificar
        self.WALKING_PROGRESS_EPSILON = 0.05  # metros — mejora mínima para contar como progreso
        self._walking_last_progress_time = None
        self._walking_last_progress_dist = None

        # --- Localización ---
        # nube del /belief: 2 métricas (spread_xy, spread_theta)
        self.CONV_XY_THRESHOLD = 0.25 # m²  (~0.5 m de desvío combinado)
        self.CONV_THETA_THRESHOLD = 0.10 # 1-R (~25° de dispersión angular)
        self.DEGRADED_XY_THRESHOLD = 1.0 # m²  (~1.0 m de desvío combinado)
        self.DEGRADED_THETA_THRESHOLD = 0.30 # 1-R (~50° de dispersión angular)
        self.MIN_PARTICLES_FOR_STATS = 3 # mínima cantidad de partículas para una varianza significativa
        self.belief_spread_xy: float | None = None # None hasta el primer /belief
        self.belief_spread_theta: float | None = None
        self.localization_converged: bool = False # flag de convergencia

        # Subscriptores ---------
        
        qos_map = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.cb_map, qos_map)
        
        self.sub_belief = self.create_subscription(PoseArray, '/belief', self.cb_belief, 10)

        self.sub_pose = self.create_subscription(PoseStamped, '/estimated_pose', self.cb_pose, 10)

        self.sub_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.cb_goal, 10)

        # BEST_EFFORT: el TB4 real publica el scan como BEST_EFFORT; una
        # suscripción RELIABLE no recibe nada. Compatible también con la sim.
        qos_sensor = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cb_scan, qos_sensor)
        

        # Publicadores ---------
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        qos_latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_path = self.create_publisher(Path, '/planned_path', qos_latched)
        # self.pub_path    = self.create_publisher(Path, '/planned_path', 1)
        self.pub_lookahead = self.create_publisher(Marker, '/lookahead_point', 10)

        # Timer para la máquina de estados ---------
        self.timer = self.create_timer(0.1, self.state_machine_loop)  # 10 Hz

    # Callbacks de subscriptores -----------------------------------------------------------------------

    def cb_map(self, msg: OccupancyGrid):
        self.map = msg
        self.inflated_map = self._inflate_map(msg)
        self.get_logger().info('Mapa recibido e inflado.')

    # def cb_pose(self, msg: PoseWithCovarianceStamped):
    #     # Convertir a PoseStamped para uniformidad (*** y que hacemos con covariance?)
    #     ps = PoseStamped()
    #     ps.header = msg.header
    #     ps.pose = msg.pose.pose
    #     self.current_pose = ps

    def cb_pose(self, msg: PoseStamped):
        self.current_pose = msg
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def cb_goal(self, msg: PoseStamped):
        self.get_logger().info(f'Nuevo goal recibido: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self.goal_pose = msg
        self.new_goal_received = True

    def cb_scan(self, msg: LaserScan):
        """
        Guarda el scan y evalúa si hay un obstáculo no mapeado en el cono frontal del robot (±45°, cono total de 90°).
 
        El LIDAR de TurtleBot3 publica ángulos en [angle_min, angle_max] con incremento angle_increment. 
        El ángulo 0 apunta hacia adelante del robot. Rangos inválidos vienen como 0.0 o inf — ambos se descartan.
        """

        # Guardo el mensaje que recibo como el último scan.
        self.last_scan = msg

        # === DEBUG ===
        # 1) Metadata del scan (una vez te basta)
        if not hasattr(self, '_scan_debug_printed'):
            self.get_logger().info(
                f'[SCAN META] frame_id={msg.header.frame_id} '
                f'angle_min={math.degrees(msg.angle_min):.1f}° '
                f'angle_max={math.degrees(msg.angle_max):.1f}° '
                f'incr={math.degrees(msg.angle_increment):.2f}° '
                f'n_rays={len(msg.ranges)}'
            )
            self._scan_debug_printed = True

        # 2) ¿Dónde está el obstáculo más cercano?
        valid = [(i, r) for i, r in enumerate(msg.ranges)
                if math.isfinite(r) and msg.range_min < r < msg.range_max]
        if valid:
            i_min, r_min = min(valid, key=lambda x: x[1])
            angle_min_obs = msg.angle_min + i_min * msg.angle_increment
            angle_norm = math.degrees(math.atan2(math.sin(angle_min_obs),
                                                math.cos(angle_min_obs)))
            self.get_logger().info(
                f'[SCAN] mín en idx={i_min} ángulo={angle_norm:+.1f}° dist={r_min:.2f}m'
            )
        # === FIN DEBUG ===
 
        # Solo detecto cuando camino:
        if self.state not in (State.WALKING,):
            return

        angle = msg.angle_min # arranco en el mínimo ángulo
        obstacle_found = False # flag
        min_front_dist = float('inf')
 
        for r in msg.ranges:
            angle_norm = math.atan2(math.sin(angle), math.cos(angle))
            
            if abs(angle_norm) <= self.CONE_HALF_ANGLE:
                if r > msg.range_min and r < msg.range_max and math.isfinite(r):
                    min_front_dist = min(min_front_dist, r)
                    if r < self.OBSTACLE_DISTANCE_THRESHOLD:
                        obstacle_found = True
                        break
 
            angle += msg.angle_increment
        
        self.get_logger().info(
        f'[SCAN] min_front={min_front_dist:.2f}m  threshold={self.OBSTACLE_DISTANCE_THRESHOLD}m  found={obstacle_found}',
        throttle_duration_sec=0.5)
 
        self.obstacle_ahead = obstacle_found

    def cb_belief(self, msg: PoseArray):
        """
        Recibe la nube de partículas del filtro (/belief) y hace dos cosas:

        Calcula la dispersión de la nube (para evaluar la confiabilidad):
             - belief_spread_xy:    var_x + var_y  (traza de la covarianza, m²)
             - belief_spread_theta: 1 - R          (dispersión circular del yaw)
           Estas métricas alimentan _check_localization_converged() y
           _localization_degraded(): una nube concentrada = pose confiable.
        """
        n = len(msg.poses)
        if n < self.MIN_PARTICLES_FOR_STATS:
            # Sin suficientes partículas no hay estadística significativa.
            self.belief_spread_xy = None
            self.belief_spread_theta = None
            return

        xs = np.array([p.position.x for p in msg.poses])
        ys = np.array([p.position.y for p in msg.poses])

        # Dispersión posicional: traza de la covarianza (suma de varianzas).
        self.belief_spread_xy = float(xs.var() + ys.var())

        # Componentes circulares del yaw (se reusan para dispersión y media).
        yaws = np.array([self._get_yaw_from_pose_msg(p) for p in msg.poses])
        cos_mean = np.cos(yaws).mean()
        sin_mean = np.sin(yaws).mean()

        # Dispersión angular: 1 - R, con R = |media de los versores| ∈ [0, 1].
        R = math.hypot(cos_mean, sin_mean)
        self.belief_spread_theta = float(1.0 - R)

    # Loop principal de la máquina de estados -----------------------------------------------------------------

    def state_machine_loop(self):

        if not self._localization_ok():
            self.get_logger().warn(
                f'Localización no confiable. Robot detenido. '
                f'Estado en espera: {self.state.name}.',
                throttle_duration_sec=2.0)
            # if self.state is not State.RELOCALIZING:
            #     self.get_logger().info('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA.')
            #     self._saved_state = self.state   # para volver después
            #     self.transition_to(State.RELOCALIZING)
            # self.run_relocalizing()
            return

        match self.state:
            case State.WAITING:
                self.run_waiting()
            case State.PLANNING:
                self.run_planning()
            case State.WALKING:
                self.run_walking()
            case State.AVOIDING:
                self.run_avoiding()
            case State.ALIGNING:
                self.run_aligning()
            # case State.RELOCALIZING:
            #     self.run_relocalizing()

    # Implementación de cada estado -----------------------------------------------------------------------

    def _localization_ok(self) -> bool:
        """
        Guard global de localización.

        True solo si la localización es confiable: el filtro convergió Y no está degradado.
        False: el loop frena el robot y no ejecuta el estado actual.
        """
        return (self._check_localization_converged("localization_ok") and not self._localization_degraded())

    def run_waiting(self):
        """
        Espera un goal del usuario.
        Transición → PLANNING cuando llega un goal.
        """

        if self.new_goal_received:
            self.new_goal_received = False
            self.get_logger().info('Goal recibido. Pasando a PLANNING.')
            self.transition_to(State.PLANNING)

    def run_planning(self):
        """
        Planifica el camino con Theta* sobre el mapa inflado.
        Transición → WALKING si encuentra camino.
        Transición → WAITING si no encuentra camino.
        """
        if self.inflated_map is None or self.current_pose is None or self.goal_pose is None:
            self.get_logger().warn('Faltan datos para planificar.')
            return

        path = self._run_theta_star(self.current_pose, self.goal_pose, self.inflated_map, self.map)

        if path:
            self.planned_path = path
            self.current_waypoint_idx = 0
            self.get_logger().info(f'Camino encontrado ({len(path)} waypoints). Pasando a WALKING.')
            self._publish_path(path)
            self._walking_last_progress_time = self.get_clock().now()
            self._walking_last_progress_dist = None
            self.transition_to(State.WALKING)
        else:
            self.get_logger().warn('No se encontró camino. Volviendo a WAITING.')
            self.transition_to(State.WAITING)

    def _walking_stuck(self) -> bool:
        """
        True si el robot lleva más de WALKING_STUCK_TIMEOUT segundos sin
        acercarse al goal (progreso = caída en la distancia al goal).

        Guarda el mejor progreso visto hasta ahora, así que ruido u
        oscilaciones chicas de Pure Pursuit no disparan un falso positivo.
        """
        if self.current_pose is None or self.goal_pose is None:
            return False

        dx = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
        dist_to_goal = math.hypot(dx, dy)

        if (self._walking_last_progress_dist is None
                or dist_to_goal < self._walking_last_progress_dist - self.WALKING_PROGRESS_EPSILON):
            self._walking_last_progress_dist = dist_to_goal
            self._walking_last_progress_time = self.get_clock().now()
            return False

        stuck_secs = (self.get_clock().now() - self._walking_last_progress_time).nanoseconds / 1e9
        if stuck_secs > self.WALKING_STUCK_TIMEOUT:
            self.get_logger().warn(
                f'WALKING sin progreso hace {stuck_secs:.1f}s '
                f'(dist_goal={dist_to_goal:.2f}m). Replanificando.')
            return True
        return False

    def run_walking(self):
        """
        Sigue el path planificado con Pure Pursuit (u otro controlador).
        Transiciones:
          → ALIGNING  si llegó al goal
          → AVOIDING  si detecta obstáculo no mapeado en el camino
          → PLANNING  si llega un goal nuevo
        """
        if self._current_path_msg:   # republicar el path en cada ciclo para que RViz lo vea
            self._current_path_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_path.publish(self._current_path_msg)

        # Prioridad 1: nuevo goal → re-planear
        if self.new_goal_received:
            self.new_goal_received = False
            self.get_logger().info('Nuevo goal durante WALKING. Pasando a PLANNING.')
            self._stop_robot()
            self.transition_to(State.PLANNING)
            return

        # Prioridad 2: obstáculo detectado en el camino
        now = self.get_clock().now().nanoseconds / 1e9
        if now >= self._avoid_cooldown_until and self._obstacle_detected_on_path():
            self.get_logger().info('Obstáculo no mapeado detectado. Pasando a AVOIDING.')
            self._stop_robot()
            self.transition_to(State.AVOIDING)
            return
        # if self._obstacle_detected_on_path():
        #     self.get_logger().info('Obstáculo detectado. Pasando a AVOIDING.')
        #     self._stop_robot()
        #     self.transition_to(State.AVOIDING)
        #     return

        # Prioridad 3: llegó al goal (posición)
        if self._reached_goal_position():
            self.get_logger().info('Posición del goal alcanzada. Pasando a ALIGNING.')
            self._stop_robot()
            self.transition_to(State.ALIGNING)
            return

        # Prioridad 4: sin progreso hacia el goal hace demasiado tiempo → replanificar
        if self._walking_stuck():
            self._stop_robot()
            self.transition_to(State.PLANNING)
            return

        # Acción normal: avanzar al siguiente waypoint con Pure Pursuit
        cmd = self._compute_pure_pursuit_cmd()
        self.pub_cmd_vel.publish(cmd)

    # def run_avoiding(self):
    #     """
    #     Esquiva un obstáculo no mapeado.
    #     Luego verifica si el próximo waypoint del path original sigue siendo
    #     alcanzable (line-of-sight libre).
    #     Transición → PLANNING.
    #     """
    #     avoidance_done = self._execute_avoidance_maneuver()

    #     if avoidance_done:
    #             self.get_logger().info('Re-planeando. Pasando a PLANNING.')
    #             self.transition_to(State.PLANNING)

    def run_avoiding(self):
        avoidance_done = self._execute_avoidance_maneuver()

        if avoidance_done:
            # Detener el robot y esperar que el PF reconverja
            self._stop_robot()

            # Si la nube está dispersa, quedarse quieto y esperar
            if (self.belief_spread_xy is not None and
                    self.belief_spread_xy > self.CONV_XY_THRESHOLD * 3):
                self.get_logger().info(
                    f'Post-AVOIDING: esperando reconvergencia PF '
                    f'(spread_xy={self.belief_spread_xy:.3f})',
                    throttle_duration_sec=0.5
                )
                return  # Quedarse en AVOIDING parado, sin transicionar

            # PF suficientemente concentrado: replanear
            now = self.get_clock().now().nanoseconds / 1e9
            self._avoid_cooldown_until = now + 1.0
            self.get_logger().info('Evasión completa + PF reconvergido. Re-planeando.')
            self.transition_to(State.PLANNING)

    def run_aligning(self):
        """
        Alinea el robot al ángulo final del goal.
        Transición → PLANNING  si llega un goal nuevo durante la alineación.
        Transición → WAITING    si la alineación terminó correctamente.
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

    # def run_relocalizing(self): # ***
    #     """Gira despacio en el lugar para alimentar al filtro con scans
    #     nuevos hasta que la nube vuelva a concentrarse."""
    #     self.get_logger().info('EN RELOCALIZING')
    #     if self._check_localization_converged("relocalizing"):
    #         self._stop_robot()
    #         self.get_logger().info('Re-localizado. Retomo PLANNING.')
    #         # replanear desde la pose actual: el path viejo ya no sirve
    #         self.transition_to(State.PLANNING)
    #         return

    #     cmd = Twist()
    #     cmd.linear.x = 0.0
    #     cmd.angular.z = 0.3   # giro suave para acumular información del scan
    #     self.pub_cmd_vel.publish(cmd)

    # Para las transiciones -----------------------------------------------------------------------

    def transition_to(self, new_state: State):
        self.get_logger().info(f'[FSM] {self.state.name} → {new_state.name}')
        self.state = new_state

    # Stubs -------------------------------------------------

    def _check_localization_converged(self, msg: str) -> bool:
        """
        True si la nube de partículas está suficientemente concentrada como
        para confiar en la pose estimada.

        Una vez que la dispersión cae por debajo de los umbrales CONVERGED, se enciende un 
        latch (self.localization_converged) que se mantiene mientras la localización no se degrade. 
        Así el robot no re-evalúa la convergencia desde cero a cada scan.

        Si todavía no llegó ningún /belief (spread None), se considera NO
        convergido: el robot espera con seguridad hasta tener una nube válida
        (típicamente tras fijar el 2D Pose Estimate en RViz).
        """
        if self.belief_spread_xy is None or self.belief_spread_theta is None:
            #self.get_logger().info('1')
            return False

        # Si ya estaba convergido, mantenerlo (la degradación se evalúa aparte).
        if self.localization_converged:
            return True

        # ambas dispersiones bajo el umbral estricto.
        if (self.belief_spread_xy < self.CONV_XY_THRESHOLD
                and self.belief_spread_theta < self.CONV_THETA_THRESHOLD):
            self.localization_converged = True
            self.get_logger().info(
                f'Localización convergió en {msg}'
                f'(spread_xy={self.belief_spread_xy:.3f} m², '
                f'spread_θ={self.belief_spread_theta:.3f}).')
            return True
        
        

        return False

    def _line_of_sight(self, grid: OccupancyGrid,
                       r0: int, c0: int,
                       r1: int, c1: int) -> bool:
        """
        Verifica si existe línea de visión libre entre dos celdas usando el algoritmo de Bresenham.

        Recorre todas las celdas que la línea recta atraviesa entre (r0,c0) y (r1,c1). Si alguna está ocupada (>= 50) o es desconocida (-1), 
        retorna False. Si todas son libres, retorna True.

        Se usa >= 50 como umbral de ocupación para tolerar ruido en el mapa.

        """
        width = grid.info.width

        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r1 > r0 else -1
        sc = 1 if c1 > c0 else -1

        r, c = r0, c0
        err = dr - dc

        while True:
            # Consulto la celda actual en el mapa inflado
            idx = r * width + c
            val = grid.data[idx]

            # Celda ocupada o desconocida -> no hay línea de visión
            if val >= 50 or val == -1:
                return False

            # Se llegó al destino
            if r == r1 and c == c1:
                return True

            # Avanzar con Bresenham
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r   += sr
            if e2 < dr:
                err += dr
                c   += sc

    def _run_theta_star(self, start: PoseStamped, goal: PoseStamped,
                        inflated_grid: OccupancyGrid, grid: OccupancyGrid) -> list:
        """
        Planifica un camino libre de colisiones entre start y goal usando Theta*.

        Retorna lista de PoseStamped (waypoints) en el frame 'map', o lista vacía si no se encontró camino.
        """
        # 1) Convertir poses a celdas:
        sx = start.pose.position.x
        sy = start.pose.position.y
        gx = goal.pose.position.x
        gy = goal.pose.position.y

        start_cell = self._world_to_grid(sx, sy, grid)
        goal_cell  = self._world_to_grid(gx, gy, grid)

        if start_cell is None or goal_cell is None:
            self.get_logger().error('Theta*: start o goal fuera del mapa.')
            return []

        rs, cs = start_cell
        rg, cg = goal_cell

        # Verificar que start y goal no estén en celda ocupada:
        width = grid.info.width
        if grid.data[rs * width + cs] >= 50:
            self.get_logger().error('Theta*: celda de inicio ocupada.')
            return []
        if grid.data[rg * width + cg] >= 50:
            self.get_logger().error('Theta*: celda de goal ocupada.')
            return []

        # 2) Heurística: distancia euclidiana.
        def h(r, c):
            return math.hypot(r - rg, c - cg)

        # 3) Estructuras de datos :
        #   g_score: costo acumulado desde el inicio
        #   parent: celda padre para reconstruir el camino
        g_score = {(rs, cs): 0.0}
        parent  = {(rs, cs): (rs, cs)}   # el inicio es su propio padre

        # Cola de prioridad: (f, row, col)
        open_set = []
        heapq.heappush(open_set, (h(rs, cs), rs, cs))
        closed_set = set()

        # 8-vecindad con sus costos (horizontal/vertical = 1, diagonal = √2)
        neighbors_offsets = [
            (-1,  0, 1.0), ( 1,  0, 1.0), ( 0, -1, 1.0), ( 0,  1, 1.0),
            (-1, -1, math.sqrt(2)), (-1,  1, math.sqrt(2)),
            ( 1, -1, math.sqrt(2)), ( 1,  1, math.sqrt(2)),
        ]

        height = inflated_grid.info.height

        # 4) Loop principal ---
        while open_set:
            _, r, c = heapq.heappop(open_set)
            current = (r, c)

            if current in closed_set:
                continue
            closed_set.add(current)

            # Llegamos al goal
            if current == (rg, cg):
                break

            pr, pc = parent[current]   # padre del nodo actual

            for dr, dc, move_cost in neighbors_offsets:
                nr, nc = r + dr, c + dc
                neighbor = (nr, nc)

                # Verificar límites del mapa
                if not (0 <= nr < height and 0 <= nc < width):
                    continue
                # Saltar si ya fue procesado
                if neighbor in closed_set:
                    continue
                # Saltar si está ocupado o es desconocido
                cell_val = inflated_grid.data[nr * width + nc]
                if cell_val >= 50 or cell_val == -1:
                    continue

                # --- Theta*: intentar conectar desde el ABUELO (parent of current) ---
                if self._line_of_sight(inflated_grid, pr, pc, nr, nc):
                    # Costo desde el abuelo al vecino directamente
                    g_via_grandparent = (g_score[(pr, pc)]
                                         + math.hypot(nr - pr, nc - pc))
                    if g_via_grandparent < g_score.get(neighbor, float('inf')):
                        g_score[neighbor] = g_via_grandparent
                        parent[neighbor]  = (pr, pc)   # saltar el nodo actual
                        f = g_via_grandparent + h(nr, nc)
                        heapq.heappush(open_set, (f, nr, nc))
                else:
                    # Sin línea de visión:
                    g_via_current = g_score[current] + move_cost
                    if g_via_current < g_score.get(neighbor, float('inf')):
                        g_score[neighbor] = g_via_current
                        parent[neighbor]  = current
                        f = g_via_current + h(nr, nc)
                        heapq.heappush(open_set, (f, nr, nc))

        # --- 5. Reconstruir camino desde goal hacia start ---
        if (rg, cg) not in parent:
            self.get_logger().warn('Theta*: no se encontró camino.')
            return []

        path_cells = []
        node = (rg, cg)
        while node != (rs, cs):
            path_cells.append(node)
            node = parent[node]
        path_cells.append((rs, cs))
        path_cells.reverse()   # start → goal

        # --- 6. Convertir celdas a PoseStamped ---
        waypoints = []
        for (row, col) in path_cells:
            wx, wy = self._grid_to_world(row, col, inflated_grid)
            waypoints.append(self._make_pose_stamped(wx, wy))

        self.get_logger().info(
            f'Theta*: camino encontrado con {len(waypoints)} waypoints.')
        return waypoints

    # -----------------------------------------------------------------------
    # Helpers:
    # -----------------------------------------------------------------------
    
    def _world_to_grid(self, x: float, y: float,
                       grid: OccupancyGrid) -> tuple[int, int] | None:
        """
        Convierte coordenadas del mundo (metros, frame 'map') a celda (row, col).
 
        El OccupancyGrid almacena las celdas en orden fila-mayor (row-major): data[row * width + col]
        El origen del mapa (grid.info.origin) es la esquina inferior-izquierda de la celda (0, 0).
 
        Retorna (row, col) si el punto cae dentro del mapa, None si está fuera.
        """
        info = grid.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
 
        if 0 <= row < info.height and 0 <= col < info.width:
            return (row, col)
 
        self.get_logger().warn(
            f'_world_to_grid: ((!) {x:.2f}, {y:.2f}) fuera del mapa.')
        return None
 
    def _grid_to_world(self, row: int, col: int,
                       grid: OccupancyGrid) -> tuple[float, float]:
        """
        Convierte celda (row, col) al centro de esa celda en metros (frame 'map').
 
        Se suma 0.5 * resolution para apuntar al centro de la celda,
        no a su esquina inferior-izquierda.
        """
        info = grid.info
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return (x, y)
 
    def _make_pose_stamped(self, x: float, y: float) -> PoseStamped:
        """Crea un PoseStamped en el frame 'map' con la posición dada."""
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0  # sin rotación por defecto
        return ps

    def _publish_path(self, waypoints: list):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = waypoints
        self._current_path_msg = path_msg
        self.pub_path.publish(path_msg)

    # def _obstacle_detected_on_path(self) -> bool:
    #     """Retorna True si cb_scan detectó algo en el cono frontal."""
    #     return self.obstacle_ahead
    def _obstacle_detected_on_path(self) -> bool:
        """Solo True si hay un obstáculo NO mapeado en el cono frontal."""
        return self.obstacle_ahead and self._obstacle_is_unmapped()

    def _obstacle_is_unmapped(self) -> bool:
        """
        Proyecta cada hit del cono frontal al mapa inflado.
        Retorna True solo si alguno cae en una celda LIBRE del mapa
        (obstáculo dinámico real), no en una celda ya ocupada (pared conocida).
        """
        if self.last_scan is None or self.inflated_map is None or self.current_pose is None:
            return False

        msg = self.last_scan
        rx = self.current_pose.pose.position.x
        ry = self.current_pose.pose.position.y
        robot_yaw = self.current_yaw
        info = self.inflated_map.info

        angle = msg.angle_min
        for r in msg.ranges:
            angle_norm = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle_norm) <= self.CONE_HALF_ANGLE:
                if (r > msg.range_min and r < msg.range_max
                        and math.isfinite(r)
                        and r < self.OBSTACLE_DISTANCE_THRESHOLD):
                    # Proyectar el hit al frame map
                    global_angle = robot_yaw + angle_norm
                    hit_x = rx + r * math.cos(global_angle)
                    hit_y = ry + r * math.sin(global_angle)

                    col = int((hit_x - info.origin.position.x) / info.resolution)
                    row = int((hit_y - info.origin.position.y) / info.resolution)

                    if 0 <= row < info.height and 0 <= col < info.width:
                        cell_val = self.inflated_map.data[row * info.width + col]
                        if cell_val < 50:
                            # Celda libre en el mapa → obstáculo no esperado
                            return True
                        # cell_val >= 50 → pared ya mapeada, ignorar
            angle += msg.angle_increment
        return False

    def _reached_goal_position(self) -> bool:
        """
        Retorna True si la distancia euclidiana entre la pose actual
        y el goal es menor a GOAL_TOLERANCE.
        """
        if self.current_pose is None or self.goal_pose is None:
            return False

        dx = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
        return math.hypot(dx, dy) < self.GOAL_TOLERANCE

    # Cambian segun entrada los siguientes 2:
    def _get_yaw_from_pose(self, pose: PoseStamped) -> float:
        """Extrae el yaw (radianes) del quaternion de una PoseStamped."""
        q = pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def _get_yaw_from_pose_msg(self, pose) -> float:
        """Extrae el yaw (rad) del quaternion de un geometry_msgs/Pose."""
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def _compute_pure_pursuit_cmd(self) -> Twist:
        """
        Implementa Pure Pursuit con velocidad lineal constante.

        Algoritmo:
          1. Buscar el primer waypoint a distancia >= LOOKAHEAD_DISTANCE.
          2. Si no existe (caso borde: robot cerca del final), usar el último
             waypoint directamente.
          3. Calcular el ángulo α entre el heading del robot y el waypoint.
          4. Calcular ω = (2 * v * sin(α)) / L  y publicar Twist.

        """
        if self.current_pose is None or not self.planned_path:
            return Twist()

        rx = self.current_pose.pose.position.x
        ry = self.current_pose.pose.position.y
        robot_yaw = self._get_yaw_from_pose(self.current_pose)

        # 1) Avanzar el índice base: descartar waypoints ya superados ---
        while self.current_waypoint_idx < len(self.planned_path) - 1:
            wp = self.planned_path[self.current_waypoint_idx]
            dx = wp.pose.position.x - rx
            dy = wp.pose.position.y - ry
            if math.hypot(dx, dy) < self.LOOKAHEAD_DISTANCE:
                # Este waypoint ya quedó atrás, avanzar al siguiente
                self.current_waypoint_idx += 1
            else:
                break

        # 2) Buscar waypoint objetivo a >= LOOKAHEAD_DISTANCE ---
        target_wp = None
        if self.current_waypoint_idx < len(self.planned_path):
            target_wp = self.planned_path[self.current_waypoint_idx]
        else:
            target_wp = self.planned_path[-1]

        # 3) Calcular α: ángulo al target relativo al heading del robot ---
        dx = target_wp.pose.position.x - rx
        dy = target_wp.pose.position.y - ry
        angle_to_target = math.atan2(dy, dx)
        alpha = math.atan2(
            math.sin(angle_to_target - robot_yaw),
            math.cos(angle_to_target - robot_yaw)
        )   # normalizado a [-π, π]

        # 4) Calcular ω con la fórmula de Pure Pursuit ---
        # ω = (2 * v * sin(α)) / L
        L = self.LOOKAHEAD_DISTANCE
        v = self.LINEAR_SPEED
        omega = (2.0 * v * math.sin(alpha)) / L

        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = omega

        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'pp'
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = target_wp.pose.position.x
        m.pose.position.y = target_wp.pose.position.y
        m.pose.position.z = 0.1
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
        self.pub_lookahead.publish(m)

        return cmd

    def _execute_avoidance_maneuver(self) -> bool:
        if self.last_scan is None:
            self._stop_robot()
            return False

        # Inicializar el yaw de referencia al comenzar la maniobra
        if self._avoid_yaw_start is None:
            self._avoid_yaw_start = self._get_yaw_from_pose(self.current_pose)
            self._avoid_turn_sign = self._pick_clearer_side()
            lado = 'izquierda' if self._avoid_turn_sign > 0 else 'derecha'
            self.get_logger().info(f'Obstáculo: girando {self.AVOID_ROTATION_ANGLE}° hacia la {lado}.')

        # ¿Cuánto giré desde el inicio de la maniobra?
        yaw_now = self._get_yaw_from_pose(self.current_pose)
        delta = abs(math.atan2(math.sin(yaw_now - self._avoid_yaw_start),
                               math.cos(yaw_now - self._avoid_yaw_start)))

        if delta >= self.AVOID_ROTATION_ANGLE:
            self._stop_robot()
            self._avoid_turn_sign = 0.0
            self._avoid_yaw_start = None   # reset para la próxima
            self.get_logger().info('Rotación de evasión completa.')
            return True

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self._avoid_turn_sign * self.AVOID_ANGULAR_SPEED
        self.pub_cmd_vel.publish(cmd)
        return False

    # def _execute_avoidance_maneuver(self) -> bool:
    #     """
    #     Política de evasión reactiva para obstáculos NO mapeados.

    #     """
    #     if self.last_scan is None:
    #         # Sin datos del LIDAR freno y espero el próximo scan.
    #         self._stop_robot()
    #         return False

    #     # 1) Chequeo de si el frente ya esta limpio:
    #     if self._front_is_clear():
    #         self._stop_robot()
    #         self._avoid_turn_sign = 0.0  # reseteo para la próxima
    #         self.get_logger().info('Frente despejado. Evasión completa.')
    #         return True

    #     # 2) Elegir hacia qué lado girar
    #     if self._avoid_turn_sign == 0.0:
    #         self._avoid_turn_sign = self._pick_clearer_side()
    #         lado = 'izquierda' if self._avoid_turn_sign > 0 else 'derecha'
    #         self.get_logger().info(f'Obstáculo: girando hacia la {lado} para esquivar.')

    #     # Girar en el lugar hacia el lado elegido
    #     cmd = Twist()
    #     cmd.linear.x = 0.0
    #     cmd.angular.z = self._avoid_turn_sign * self.AVOID_ANGULAR_SPEED
    #     self.pub_cmd_vel.publish(cmd)

    #     return False

    # def _front_is_clear(self) -> bool:
    #     """
    #     True si NINGÚN rayo válido del cono frontal (±CONE_HALF_ANGLE) está más
    #     cerca que OBSTACLE_DISTANCE_THRESHOLD * AVOID_CLEAR_MARGIN.

    #     El margen extra es para evitar declarar "libre" cuando el obstáculo apenas salió del umbral de detección.
    #     """
    #     msg = self.last_scan
    #     clear_dist = self.OBSTACLE_DISTANCE_THRESHOLD * self.AVOID_CLEAR_MARGIN

    #     angle = msg.angle_min
    #     for r in msg.ranges:
    #         angle_norm = math.atan2(math.sin(angle), math.cos(angle))
    #         if abs(angle_norm) <= self.CONE_HALF_ANGLE:
    #             if (r > msg.range_min and r < msg.range_max
    #                     and math.isfinite(r) and r < clear_dist):
    #                 return False
    #         angle += msg.angle_increment
    #     return True

    def _pick_clearer_side(self) -> float:
        """
        Mira el sector ±AVOID_EVAL_HALF_ANGLE y devuelve +1.0 si conviene girar a
        la izquierda (ángulos positivos del LIDAR) o -1.0 a la derecha, según qué
        lado tenga MAYOR distancia libre mínima (el lado con el rayo más cercano
        es el más bloqueado, así que se gira hacia el opuesto).

        Si un lado no tiene lecturas válidas, se elige el otro. Si ninguno tiene,
        se gira a la izquierda por defecto.
        """
        msg = self.last_scan
        min_left = float('inf') # ángulos positivos (izquierda)
        min_right = float('inf') # ángulos negativos (derecha)

        angle = msg.angle_min
        for r in msg.ranges:
            angle_norm = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle_norm) <= self.AVOID_EVAL_HALF_ANGLE:
                if r > msg.range_min and r < msg.range_max and math.isfinite(r):
                    if angle_norm > 0:
                        min_left = min(min_left, r)
                    elif angle_norm < 0:
                        min_right = min(min_right, r)
            angle += msg.angle_increment

        if min_left >= min_right:
            return +1.0
        else:
            return -1.0
        
    def _align_to_goal_angle(self) -> bool:
        """
        Gira el robot hasta alcanzar el yaw del goal con tolerancia ±5°.

        Calcula el error angular entre el yaw actual y el yaw deseado,
        normaliza a [-π, π] para siempre girar por el camino más corto,
        y publica un Twist solo con velocidad angular.

        Retorna True cuando el error es menor a ANGLE_TOLERANCE.
        """
        if self.current_pose is None or self.goal_pose is None:
            return False
        
        if self.new_goal_received:
            return True

        # Yaw actual del robot
        current_yaw = self._get_yaw_from_pose(self.current_pose)

        # Yaw deseado: extraído del quaternion del goal
        goal_yaw = self._get_yaw_from_pose(self.goal_pose)

        # Error angular normalizado a [-π, π]
        error = math.atan2(
            math.sin(goal_yaw - current_yaw),
            math.cos(goal_yaw - current_yaw)
        )

        if abs(error) < self.ANGLE_TOLERANCE:
            # Alineado: frenar rotación y señalizar fin
            self._stop_robot()
            self.get_logger().info(
                f'Alineación completa. Error residual: {math.degrees(error):.1f}°')
            return True

        # Girar en la dirección correcta con velocidad constante
        cmd = Twist()
        cmd.angular.z = math.copysign(self.ANGULAR_SPEED, error)
        self.pub_cmd_vel.publish(cmd)
        return False

    def _localization_degraded(self) -> bool:
        """
        True si la nube de partículas se dispersó tanto durante el recorrido
        que la pose ya no es confiable.

        """

        if self.belief_spread_xy is None or self.belief_spread_theta is None:
            #self.get_logger().info('3')
            return False

        degraded = (self.belief_spread_xy > self.DEGRADED_XY_THRESHOLD
                    or self.belief_spread_theta > self.DEGRADED_THETA_THRESHOLD)

        if degraded and self.localization_converged:
            # Romper el latch: la próxima vez habrá que volver a converger.
            self.localization_converged = False
            self.get_logger().warn(
                f'Localización DEGRADADA '
                f'(spread_xy={self.belief_spread_xy:.3f} m², '
                f'spread_θ={self.belief_spread_theta:.3f}). '
                f'Robot detenido hasta re-converger.')
        return degraded
    
    def _inflate_map(self, grid: OccupancyGrid) -> OccupancyGrid:
        """
        Expande cada celda ocupada del mapa por un radio de INFLATION_RADIUS_CELLS celdas usando una máscara circular (distancia euclidiana).
 
        Celdas del OccupancyGrid:
          -1  → desconocido  (no se toca)
           0  → libre
         100  → ocupado
 
        Todas las celdas dentro del radio de una celda ocupada
        pasan a valer 100 (excepto las desconocidas).
        """

        r = self.INFLATION_RADIUS_CELLS # radio por el cual inflo
        width  = grid.info.width # tomo medida de ancho
        height = grid.info.height # tomo medida de alto
 
        # Convierto el mapa a array 2D numpy para operar eficientemente
        raw = np.array(grid.data, dtype=np.int8).reshape((height, width))
 
        # Máscara de las celdas ocupadas:
        occupied = (raw == 100)
 
        # Kernel circular de radio r:
        y_k, x_k = np.ogrid[-r:r+1, -r:r+1]
        kernel = (x_k**2 + y_k**2) <= r**2 # true dentro del círculo
 
        # Dilato la máscara de ocupados con el kernel circular:
        dilated = binary_dilation(occupied, structure=kernel)
 
        # Aplico la dilatación: solo inflar celdas que eran libres (0), no tocar las desconocidas (-1)
        inflated_raw = raw.copy()
        inflate_mask = dilated & (raw == 0) # libre ahora pasa a ocupado
        inflated_raw[inflate_mask] = 100
 
        # Reconstruir el OccupancyGrid con los mismos metadatos:
        inflated_grid = copy.deepcopy(grid)
        inflated_grid.data = inflated_raw.flatten().tolist()
 
        self.get_logger().info(
            f'Mapa inflado: {int(inflate_mask.sum())} celdas nuevas bloqueadas '
            f'(radio={r} celdas = {r * grid.info.resolution:.2f} m).'
        )
        return inflated_grid

    def _stop_robot(self):
        self.pub_cmd_vel.publish(Twist()) # Twist vacío = stop


# ---------------------------------------------------------------------------

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