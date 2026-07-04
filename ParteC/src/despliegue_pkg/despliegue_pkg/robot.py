#!/usr/bin/env python3
"""
Nodo de navegación autónoma - Esqueleto con máquina de estados.
Estados: WAITING → PLANNING → WALKING → AVOIDING → ALIGNING → WAITING
"""
import heapq
import math
import numpy as np
import copy
from scipy.ndimage import binary_dilation, distance_transform_edt, label
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from enum import Enum, auto

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
from sensor_msgs.msg import LaserScan


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
        self.current_yaw = 0.0

        # --- Detección de obstáculos ---
        self.last_scan: LaserScan | None = None
        self.obstacle_ahead: bool = False # flag crudo (sin debounce) de si hay un obstaculo delante
        self.OBSTACLE_DISTANCE_THRESHOLD = 0.15 # en metros
        self.CONE_HALF_ANGLE = math.radians(45) # ±45°: un cono delantero de un total de 90°

        # Debounce/histéresis: una lectura aislada no debe disparar ni cancelar
        # una evasión (el sensor puede parpadear). Se confirma/descarta recién
        # tras N lecturas consecutivas en el mismo sentido.
        self.OBSTACLE_CONFIRM_SCANS = 3
        self.OBSTACLE_CLEAR_SCANS = 3
        self._obstacle_hit_streak = 0
        self._obstacle_clear_streak = 0
        self.obstacle_confirmed: bool = False

        # Rayo más cercano dentro del cono frontal (robot-frame), para ubicar
        # el obstáculo dinámico en el mapa al replanificar alrededor suyo.
        self._closest_obstacle_range: float | None = None
        self._closest_obstacle_bearing: float | None = None

        # --- Evasión de obstáculos no mapeados ---
        self.AVOID_EVAL_HALF_ANGLE = math.radians(90)  # +-90°: sector que se mira para elegir lado
        self._avoiding_dynamic_obstacle: bool = False  # PLANNING debe usar mapa aumentado

        # --- Inflar el mapa ---
        self.inflated_map: OccupancyGrid | None = None
        self.INFLATION_RADIUS_CELLS = 3  # 3 celdas × 0.05 m = 0.15 m de margen
        # Pasajes angostos: dos obstáculos a menos de 2×INFLATION_RADIUS
        # quedan sellados al inflar aunque el robot pase físicamente. En
        # inflated_map_reopened la línea media de esos pasajes está reabierta
        # (si el despeje real es >= NARROW_REOPEN_MIN_CELLS); se usa SOLO como
        # fallback cuando la planificación normal no encuentra camino.
        # Si el robot roza al cruzar un pasaje, subir el radio; si un pasaje
        # transitable sigue cerrado, bajarlo (mín 1).
        self.inflated_map_reopened: OccupancyGrid | None = None
        self.NARROW_REOPEN_MIN_CELLS = 2  # 2 celdas × 0.05 m = 0.10 m por lado

        # --- Pure Pursuit ---
        self.LOOKAHEAD_DISTANCE = 0.4   # metros
        self.LINEAR_SPEED       = 0.15  # m/s — constante
        self.GOAL_TOLERANCE     = 0.10  # metros — distancia para considerar que llegó
        self.ANGLE_TOLERANCE    = math.radians(12.0)  # ±5° para considerar alineado
        self.ANGULAR_SPEED      = 0.1   # rad/s — velocidad de giro en ALIGNING

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

        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cb_scan, 10)
        

        # Publicadores ---------
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path    = self.create_publisher(Path, '/planned_path', 1)

        # Timer para la máquina de estados ---------
        self.timer = self.create_timer(0.1, self.state_machine_loop)  # 10 Hz

    # Callbacks de subscriptores -----------------------------------------------------------------------

    def cb_map(self, msg: OccupancyGrid):
        self.map = msg
        self.inflated_map = self._inflate_map(msg)
        # Variante de fallback con los pasajes angostos reabiertos (ver
        # run_planning): misma inflación, pero la línea media de los pasajes
        # que quedaron sellados vuelve a ser transitable.
        self.inflated_map_reopened = self._inflate_map(msg, reopen_narrow=True)
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

        Corre en todos los estados (no solo WALKING): PLANNING necesita la
        lectura más fresca posible para re-chequear el obstáculo apenas
        Theta* termina de calcular el camino alternativo (ver
        _run_planning_around_obstacle).
        """

        # Guardo el mensaje que recibo como el último scan.
        self.last_scan = msg

        angle = msg.angle_min # arranco en el mínimo ángulo
        closest_range = None
        closest_bearing = None

        for r in msg.ranges:
            # Normalizo el ángulo al rango [-π, π]
            angle_norm = math.atan2(math.sin(angle), math.cos(angle))

            # Me pregunto si está dentro del cono frontal de ±45°:
            if abs(angle_norm) <= self.CONE_HALF_ANGLE:
                # Descarto lecturas inválidas (0.0, inf, nan)
                if (r > msg.range_min
                        and r < msg.range_max
                        and math.isfinite(r)
                        and r < self.OBSTACLE_DISTANCE_THRESHOLD):
                    if closest_range is None or r < closest_range:
                        closest_range = r
                        closest_bearing = angle_norm

            angle += msg.angle_increment

        obstacle_found = closest_range is not None
        self.obstacle_ahead = obstacle_found
        self._closest_obstacle_range = closest_range
        self._closest_obstacle_bearing = closest_bearing

        # Debounce: confirma/descarta el obstáculo tras N lecturas seguidas
        # en el mismo sentido, para no oscilar por una detección aislada.
        if obstacle_found:
            self._obstacle_hit_streak += 1
            self._obstacle_clear_streak = 0
            if self._obstacle_hit_streak >= self.OBSTACLE_CONFIRM_SCANS:
                self.obstacle_confirmed = True
        else:
            self._obstacle_clear_streak += 1
            self._obstacle_hit_streak = 0
            if self._obstacle_clear_streak >= self.OBSTACLE_CLEAR_SCANS:
                self.obstacle_confirmed = False

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
        self.get_logger().info(f"[WAITING STATE] new_goal_received: {self.new_goal_received}")

        if self.new_goal_received:
            self.new_goal_received = False
            self.get_logger().info('Goal recibido. Pasando a PLANNING.')
            self.transition_to(State.PLANNING)

    def run_planning(self):
        """
        Planifica el camino con Theta* sobre el mapa inflado.
        Si se llegó acá desde AVOIDING (obstáculo dinámico), delega en
        _run_planning_around_obstacle en vez del flujo normal.
        Transición → WALKING si encuentra camino.
        Transición → WAITING si no encuentra camino.
        """
        if self.inflated_map is None or self.current_pose is None or self.goal_pose is None:
            self.get_logger().warn('Faltan datos para planificar.')
            return

        if self._avoiding_dynamic_obstacle:
            self._run_planning_around_obstacle()
            return

        path = self._run_theta_star(self.current_pose, self.goal_pose, self.inflated_map)

        # Fallback: si la inflación selló el único pasaje hacia el goal
        # (dos obstáculos muy cercanos), reintentar con el mapa que tiene
        # la línea media de esos pasajes reabierta. Solo se llega acá si
        # NO existe ningún camino con el margen completo.
        if not path and self.inflated_map_reopened is not None:
            self.get_logger().warn(
                'Sin camino en el mapa inflado: reintento con pasajes '
                'angostos reabiertos (margen reducido al cruzarlos).')
            path = self._run_theta_star(self.current_pose, self.goal_pose,
                                        self.inflated_map_reopened)

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

    def _run_planning_around_obstacle(self):
        """
        Segundo modo de PLANNING, disparado desde AVOIDING.

        Marca el obstáculo dinámico sobre una copia del mapa inflado
        (self.inflated_map no se toca — el mapa base queda intacto para la
        próxima planificación) y corre Theta* sobre esa copia. Theta* es
        síncrono, así que para cuando termina de calcular ya pasó tiempo
        suficiente como para re-chequear el sensor (mismo debounce que
        dispara AVOIDING) y decidir:
          - Obstáculo sigue confirmado → adopta el camino nuevo (rodeo).
          - Ya no está → descarta el camino nuevo y retoma el viejo tal cual
            (mismo índice de waypoint), sin gastar otra vuelta de Theta*.

        Al volver a WALKING resetea el debounce: si no, el obstáculo (que
        sigue físicamente en el mismo lugar, el robot no se movió durante
        PLANNING) seguiría "confirmado" en el primer tick y retriggerearía
        AVOIDING de inmediato.
        """
        self._avoiding_dynamic_obstacle = False

        obstacle_xy = self._get_obstacle_map_position()
        if obstacle_xy is not None:
            planning_grid = self._make_augmented_map(self.inflated_map, obstacle_xy)
        else:
            planning_grid = self.inflated_map

        alt_path = self._run_theta_star(self.current_pose, self.goal_pose, planning_grid)

        # Mismo fallback que run_planning: si con el margen completo no hay
        # rodeo posible, reintentar sobre el mapa con pasajes reabiertos
        # (el obstáculo dinámico se vuelve a marcar sobre esa base).
        if not alt_path and self.inflated_map_reopened is not None:
            self.get_logger().warn(
                'Sin rodeo en el mapa inflado: reintento con pasajes '
                'angostos reabiertos.')
            if obstacle_xy is not None:
                planning_grid = self._make_augmented_map(
                    self.inflated_map_reopened, obstacle_xy)
            else:
                planning_grid = self.inflated_map_reopened
            alt_path = self._run_theta_star(self.current_pose, self.goal_pose,
                                            planning_grid)

        if self.obstacle_confirmed:
            if alt_path:
                self.planned_path = alt_path
                self.current_waypoint_idx = 0
                self._publish_path(alt_path)
                self.get_logger().info(
                    f'Obstáculo sigue ahí: camino nuevo ({len(alt_path)} waypoints) rodeándolo.')
            else:
                self.get_logger().warn(
                    'Obstáculo sigue ahí y no se encontró camino alternativo. Volviendo a WAITING.')
                self._reset_obstacle_debounce()
                self.transition_to(State.WAITING)
                return
        else:
            self.get_logger().info('Obstáculo ya no está. Retomando el camino anterior.')

        self._reset_obstacle_debounce()
        self._walking_last_progress_time = self.get_clock().now()
        self._walking_last_progress_dist = None
        self.transition_to(State.WALKING)

    def _reset_obstacle_debounce(self):
        """Limpia el debounce de obstáculo al salir de PLANNING-por-evasión."""
        self.obstacle_confirmed = False
        self._obstacle_hit_streak = 0
        self._obstacle_clear_streak = 0

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
        # Prioridad 1: nuevo goal → re-planear
        if self.new_goal_received:
            self.new_goal_received = False
            self.get_logger().info('Nuevo goal durante WALKING. Pasando a PLANNING.')
            self._stop_robot()
            self.transition_to(State.PLANNING)
            return

        # Prioridad 2: obstáculo detectado en el camino
        if self._obstacle_detected_on_path():
            self.get_logger().info('Obstáculo detectado. Pasando a AVOIDING.')
            self._stop_robot()
            self.transition_to(State.AVOIDING)
            return

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

    def run_avoiding(self):
        """
        Frena ante un obstáculo no mapeado. No gira a ciegas: delega en
        PLANNING el cálculo de un camino alternativo con el obstáculo
        marcado sobre una copia del mapa (ver _run_planning_around_obstacle).
        Transición → PLANNING.
        """
        self._stop_robot()
        self._avoiding_dynamic_obstacle = True
        self.get_logger().info('Obstáculo confirmado. Replanificando con mapa aumentado.')
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
                        grid: OccupancyGrid) -> list:
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

        # Si start/goal caen dentro del margen de seguridad, reubicar a la
        # celda libre más cercana en vez de fallar directo -- común cuando el
        # robot está pegado a una pared o el goal se pide muy cerca de un
        # obstáculo (incluye el obstáculo dinámico marcado por
        # _make_augmented_map, que ya se cuida de no tapar la celda de inicio,
        # pero esto es una red de seguridad adicional para el resto de casos).
        height_check = grid.info.height

        def _grid_free(r, c):
            if not (0 <= r < height_check and 0 <= c < width):
                return False
            v = grid.data[r * width + c]
            return 0 <= v < 50

        def _nearest_free(r, c, max_radius=40):
            if _grid_free(r, c):
                return (r, c)
            from collections import deque
            seen = {(r, c)}
            q = deque([(r, c, 0)])
            while q:
                cr, cc, d = q.popleft()
                if d >= max_radius:
                    continue
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1),
                               (-1, -1), (-1, 1), (1, -1), (1, 1)):
                    nr, nc = cr + dr, cc + dc
                    if (nr, nc) in seen:
                        continue
                    seen.add((nr, nc))
                    if _grid_free(nr, nc):
                        return (nr, nc)
                    if 0 <= nr < height_check and 0 <= nc < width:
                        q.append((nr, nc, d + 1))
            return None

        if not _grid_free(rs, cs):
            free = _nearest_free(rs, cs)
            if free is None:
                self.get_logger().warn('Theta*: inicio encerrado en zona inflada, sin celda libre cerca.')
                return []
            self.get_logger().info(f'Theta*: inicio reubicado de ({rs},{cs}) a celda libre {free}.')
            rs, cs = free
        if not _grid_free(rg, cg):
            free = _nearest_free(rg, cg)
            if free is None:
                self.get_logger().warn('Theta*: goal encerrado en zona inflada, sin celda libre cerca.')
                return []
            self.get_logger().info(f'Theta*: goal reubicado de ({rg},{cg}) a celda libre {free}.')
            rg, cg = free

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

        height = grid.info.height

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
                cell_val = grid.data[nr * width + nc]
                if cell_val >= 50 or cell_val == -1:
                    continue

                # --- Theta*: intentar conectar desde el ABUELO (parent of current) ---
                if self._line_of_sight(grid, pr, pc, nr, nc):
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
            wx, wy = self._grid_to_world(row, col, grid)
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
        self.pub_path.publish(path_msg)

    def _obstacle_detected_on_path(self) -> bool:
        """Retorna True si el sensor confirmó (con debounce) algo en el cono frontal."""
        return self.obstacle_confirmed

    def _get_obstacle_map_position(self) -> tuple[float, float] | None:
        """
        Convierte la lectura más cercana del cono frontal (robot-frame) al
        frame 'map', usando la pose actual. None si no hay lectura válida.
        """
        if (self._closest_obstacle_range is None
                or self._closest_obstacle_bearing is None
                or self.current_pose is None):
            return None

        r = self._closest_obstacle_range
        bearing = self._closest_obstacle_bearing

        # robot-frame: x adelante, y izquierda (REP103)
        ox = r * math.cos(bearing)
        oy = r * math.sin(bearing)

        robot_x = self.current_pose.pose.position.x
        robot_y = self.current_pose.pose.position.y
        yaw = self._get_yaw_from_pose(self.current_pose)

        map_x = robot_x + ox * math.cos(yaw) - oy * math.sin(yaw)
        map_y = robot_y + ox * math.sin(yaw) + oy * math.cos(yaw)
        return (map_x, map_y)

    def _make_augmented_map(self, grid: OccupancyGrid,
                            obstacle_xy: tuple[float, float]) -> OccupancyGrid:
        """
        Copia temporal del mapa con el obstáculo dinámico marcado como
        ocupado (mismo radio que INFLATION_RADIUS_CELLS, para que Theta* le
        deje el mismo margen que a una pared). No modifica `grid`: el mapa
        base (self.inflated_map) queda intacto para la próxima planificación.
        """
        cell = self._world_to_grid(obstacle_xy[0], obstacle_xy[1], grid)
        if cell is None:
            self.get_logger().warn('Obstáculo fuera del mapa, no se puede marcar.')
            return grid

        row, col = cell
        width, height = grid.info.width, grid.info.height
        r = self.INFLATION_RADIUS_CELLS

        augmented = copy.deepcopy(grid)
        raw = np.array(augmented.data, dtype=np.int8).reshape((height, width))

        rows_idx, cols_idx = np.ogrid[:height, :width]
        to_mark = ((rows_idx - row) ** 2 + (cols_idx - col) ** 2) <= r ** 2
        to_mark &= (raw != -1)  # no tocar celdas desconocidas

        # Nunca tapar al propio robot: OBSTACLE_DISTANCE_THRESHOLD (0.15 m) es
        # menor que el radio de inflación (0.30 m), así que el círculo del
        # obstáculo casi siempre alcanza la celda de inicio y Theta* fallaría
        # antes de arrancar ("celda de inicio ocupada"). Se descarta del
        # marcado un disco del MISMO radio r centrado en el robot: como son
        # dos discos de igual radio con centros distintos, el del robot
        # siempre sobresale del disco del obstáculo hacia el lado opuesto,
        # garantizando al menos una salida para Theta*.
        start_cell = self._world_to_grid(self.current_pose.pose.position.x,
                                         self.current_pose.pose.position.y, grid)
        if start_cell is not None:
            srow, scol = start_cell
            keep_clear = ((rows_idx - srow) ** 2 + (cols_idx - scol) ** 2) <= r ** 2
            to_mark &= ~keep_clear

        raw[to_mark] = 100
        augmented.data = raw.flatten().tolist()
        return augmented

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
        return cmd

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
                if (r > msg.range_min and r < msg.range_max and math.isfinite(r)):
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
    
    def _inflate_map(self, grid: OccupancyGrid,
                     reopen_narrow: bool = False) -> OccupancyGrid:
        """
        Expande cada celda ocupada del mapa por un radio de INFLATION_RADIUS_CELLS celdas usando una máscara circular (distancia euclidiana).

        Celdas del OccupancyGrid:
          -1  → desconocido  (no se toca)
           0  → libre
         100  → ocupado

        Todas las celdas dentro del radio de una celda ocupada
        pasan a valer 100 (excepto las desconocidas).

        Con ``reopen_narrow=True`` devuelve la variante de fallback: la
        línea media de los pasajes que la inflación selló queda transitable
        (ver _reopen_narrow_passages).
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

        # Solo para la variante de fallback: reabrir la línea media de los
        # pasajes angostos que la inflación haya sellado.
        reopened = 0
        if reopen_narrow:
            reopened = self._reopen_narrow_passages(raw, inflated_raw)
 
        # Reconstruir el OccupancyGrid con los mismos metadatos:
        inflated_grid = copy.deepcopy(grid)
        inflated_grid.data = inflated_raw.flatten().tolist()
 
        self.get_logger().info(
            f'Mapa inflado: {int(inflate_mask.sum())} celdas nuevas bloqueadas '
            f'(radio={r} celdas = {r * grid.info.resolution:.2f} m); '
            f'{reopened} celdas reabiertas en pasajes angostos.'
        )
        return inflated_grid

    def _reopen_narrow_passages(self, raw: np.ndarray,
                                inflated_raw: np.ndarray) -> int:
        """
        Reabre la línea media de los pasajes que la inflación selló.

        Dos obstáculos separados por menos de 2×INFLATION_RADIUS quedan
        "pegados" al inflar y Theta* no encuentra camino, aunque el robot
        entre físicamente (las paredes del mapa además están engordadas por
        el blur del SLAM). Bajar la inflación global no sirve: el camino
        abrazaría TODAS las esquinas al radio reducido. En cambio se reabren
        solo las celdas selladas que cumplen:

          1. eran libres en el mapa real (la inflación las tapó), y
          2. tienen despeje real >= NARROW_REOPEN_MIN_CELLS (transformada de
             distancia euclidiana al obstáculo real más cercano), y
          3. son cresta del pasaje: máximo local de esa distancia a lo largo
             de algún eje (la línea media entre los dos obstáculos), y
          4. conectan dos regiones libres DISTINTAS del mapa inflado. Esto
             descarta la bisectriz de las esquinas cóncavas (también es
             máximo local de la distancia — eje medial — pero une una región
             consigo misma): en las esquinas se conserva el margen completo.

        El camino resultante cruza el pasaje centrado, con el máximo despeje
        físicamente posible. Modifica ``inflated_raw`` in place y devuelve
        la cantidad de celdas reabiertas.
        """
        r_min = self.NARROW_REOPEN_MIN_CELLS
        occupied = (raw == 100)
        if not occupied.any():
            return 0

        # Distancia (en celdas) de cada celda al obstáculo real más cercano
        edt = distance_transform_edt(~occupied)

        # Celdas selladas por la inflación con despeje real suficiente
        sealed = (inflated_raw == 100) & (raw == 0) & (edt >= r_min)
        if not sealed.any():
            return 0

        # Cresta: máximo local de edt según algún eje (H, V o diagonales).
        # El test asimétrico (> de un lado, >= del otro) evita marcar las
        # bandas de distancia constante paralelas a una pared recta, pero sí
        # captura el centro de un pasaje de ancho par (empate en el medio).
        ridge = np.zeros_like(sealed)
        for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
            n_prev = np.roll(np.roll(edt, dr, axis=0), dc, axis=1)
            n_next = np.roll(np.roll(edt, -dr, axis=0), -dc, axis=1)
            ridge |= (edt > n_prev) & (edt >= n_next)

        # Engordar la cresta 1 celda (sin salir de lo sellado) para que
        # quede 8-conexa y la línea de visión de Theta* pueda atravesarla.
        ridge = binary_dilation(ridge & sealed) & sealed
        if not ridge.any():
            return 0

        # Requisito 4: quedarse solo con los grupos de cresta que unen dos
        # componentes libres distintas del mapa inflado. La bisectriz de una
        # esquina cóncava toca una sola componente y se descarta.
        eight = np.ones((3, 3), dtype=bool)
        free_labels, _ = label(inflated_raw == 0, structure=eight)
        ridge_groups, n_groups = label(ridge, structure=eight)
        reopened = 0
        for i in range(1, n_groups + 1):
            group = (ridge_groups == i)
            # Componentes libres adyacentes al grupo (halo de 1 celda)
            halo = binary_dilation(group) & (inflated_raw == 0)
            touched = np.unique(free_labels[halo])
            touched = touched[touched != 0]
            if touched.size >= 2:
                inflated_raw[group] = 0
                reopened += int(group.sum())
        return reopened

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