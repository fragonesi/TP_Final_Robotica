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
from enum import Enum, auto

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan


# ---------------------------------------------------------------------------
# Definición de estados
# ---------------------------------------------------------------------------

class State(Enum):
    WAITING    = auto()   # Espera un goal
    PLANNING   = auto()   # Planifica el camino con Theta*
    WALKING    = auto()   # Sigue el path planificado
    AVOIDING   = auto()   # Esquiva un obstáculo no mapeado
    ALIGNING   = auto()   # Alinea el ángulo final al llegar al goal


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

        # --- Detección de obstáculos ---
        self.last_scan: LaserScan | None = None
        self.obstacle_ahead: bool = False # flag de si hay un obstaculo delante
        self.OBSTACLE_DISTANCE_THRESHOLD = 0.4 # en metros
        self.CONE_HALF_ANGLE = math.radians(45) # ±45°: un cono delantero de un total de 90°

        # --- Inflar el mapa ---
        self.inflated_map: OccupancyGrid | None = None
        self.INFLATION_RADIUS_CELLS = 3  # 3 celdas × 0.05 m = 0.15 m de margen

        # --- Pure Pursuit ---
        self.LOOKAHEAD_DISTANCE = 0.4   # metros
        self.LINEAR_SPEED       = 0.15  # m/s — constante
        self.GOAL_TOLERANCE     = 0.10  # metros — distancia para considerar que llegó
        self.ANGLE_TOLERANCE    = math.radians(5.0)  # ±5° para considerar alineado
        self.ANGULAR_SPEED      = 0.3   # rad/s — velocidad de giro en ALIGNING

        # Subscriptores ---------
        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self.cb_map, 1)

        # self.sub_pose = self.create_subscription(
        #     PoseWithCovarianceStamped, '/amcl_pose', self.cb_pose, 10)
        self.sub_pose = self.create_subscription(Odometry, "/calc_odom", self.cb_pose, 10)

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
        self.get_logger().info('Mapa recibido e inflado.')

    # def cb_pose(self, msg: PoseWithCovarianceStamped):
    #     # Convertir a PoseStamped para uniformidad (*** y que hacemos con covariance?)
    #     ps = PoseStamped()
    #     ps.header = msg.header
    #     ps.pose = msg.pose.pose
    #     self.current_pose = ps

    def cb_pose(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.current_pose = ps

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
 
        # Solo detecto cuando camino:
        if self.state not in (State.WALKING,):
            return

        angle = msg.angle_min # arranco en el mínimo ángulo
        obstacle_found = False # flag
 
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
                    obstacle_found = True
                    break 
 
            angle += msg.angle_increment
 
        self.obstacle_ahead = obstacle_found

    # Loop principal de la máquina de estados -----------------------------------------------------------------

    def state_machine_loop(self):

        if not self._localization_ok():
            self._stop_robot()
            self.get_logger().warn(
                f'Localización no confiable. Robot detenido. '
                f'Estado en espera: {self.state.name}.',
                throttle_duration_sec=2.0)
            return

        match self.state:
            case State.WAITING:
                self.run_waiting()
            case State.PLANNING:
                self.run_planning() # Yo
            case State.WALKING:
                self.run_walking() # Yo
            case State.AVOIDING:
                self.run_avoiding()
            case State.ALIGNING:
                self.run_aligning()

    # Implementación de cada estado -----------------------------------------------------------------------

    def _localization_ok(self) -> bool:
        """
        Guard global de localización.

        True solo si la localización es confiable: el filtro convergió Y no está degradado.
        False: el loop frena el robot y no ejecuta el estado actual.
        """
        return (self._check_localization_converged()
                and not self._localization_degraded())

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

        path = self._run_theta_star(self.current_pose, self.goal_pose, self.inflated_map)

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

        # Acción normal: avanzar al siguiente waypoint con Pure Pursuit
        cmd = self._compute_pure_pursuit_cmd()
        self.pub_cmd_vel.publish(cmd)

    def run_avoiding(self):
        """
        Esquiva un obstáculo no mapeado.
        Luego verifica si el próximo waypoint del path original sigue siendo
        alcanzable (line-of-sight libre).
        Transición → PLANNING.
        """
        avoidance_done = self._execute_avoidance_maneuver()

        if avoidance_done:
                self.get_logger().info('Re-planeando. Pasando a PLANNING.')
                self.transition_to(State.PLANNING)

    def run_aligning(self):
        """
        Alinea el robot al ángulo final del goal.
        Transición → PLANNING  si llega un goal nuevo durante la alineación.
        Transición → WAITING    si la alineación terminó correctamente.
        """
        alignment_done = self._align_to_goal_angle()

        if alignment_done:
            self._stop_robot()
            if self.new_goal_received:
                self.new_goal_received = False
                self.get_logger().info('Nuevo goal durante ALIGNING. Pasando a PLANNING.')
                self.transition_to(State.PLANNING)
            else:
                self.get_logger().info('Alineación completa. Pasando a WAITING.')
                self.transition_to(State.WAITING)

    # Para las transiciones -----------------------------------------------------------------------

    def transition_to(self, new_state: State):
        self.get_logger().info(f'[FSM] {self.state.name} → {new_state.name}')
        self.state = new_state

    # Stubs -------------------------------------------------

    def _check_localization_converged(self) -> bool:
        """TODO: verificar varianza de partículas AMCL o covarianza EKF.

        Consideramos la localización válida simplemente cuando ya
        recibimos al menos una medición de odometría.
        """

        return self.current_pose is not None

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
    # Helpers de conversión: mundo ↔ grilla
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
        """Retorna True si cb_scan detectó algo en el cono frontal."""
        return self.obstacle_ahead

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

    def _get_yaw_from_pose(self, pose: PoseStamped) -> float:
        """Extrae el yaw (radianes) del quaternion de una PoseStamped."""
        q = pose.pose.orientation
        # Fórmula yaw desde quaternion (roll y pitch asumidos 0)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    # ***
    def _compute_pure_pursuit_cmd(self) -> Twist:
        """
        Implementa Pure Pursuit con velocidad lineal constante.

        Algoritmo:
          1. Buscar el primer waypoint a distancia >= LOOKAHEAD_DISTANCE.
          2. Si no existe (caso borde: robot cerca del final), usar el último
             waypoint directamente.
          3. Calcular el ángulo α entre el heading del robot y el waypoint.
          4. Calcular ω = (2 * v * sin(α)) / L  y publicar Twist.

        El índice current_waypoint_idx se avanza para no re-examinar
        waypoints ya superados, lo que hace la búsqueda O(1) amortizado.
        """
        if self.current_pose is None or not self.planned_path:
            return Twist()

        rx = self.current_pose.pose.position.x
        ry = self.current_pose.pose.position.y
        robot_yaw = self._get_yaw_from_pose(self.current_pose)

        # --- 1. Avanzar el índice base: descartar waypoints ya superados ---
        while self.current_waypoint_idx < len(self.planned_path) - 1:
            wp = self.planned_path[self.current_waypoint_idx]
            dx = wp.pose.position.x - rx
            dy = wp.pose.position.y - ry
            if math.hypot(dx, dy) < self.LOOKAHEAD_DISTANCE:
                # Este waypoint ya quedó atrás, avanzar al siguiente
                self.current_waypoint_idx += 1
            else:
                break

        # --- 2. Buscar waypoint objetivo a >= LOOKAHEAD_DISTANCE ---
        target_wp = None
        if self.current_waypoint_idx < len(self.planned_path):
            target_wp = self.planned_path[self.current_waypoint_idx]
        else:
            target_wp = self.planned_path[-1]

        # --- 3. Calcular α: ángulo al target relativo al heading del robot ---
        dx = target_wp.pose.position.x - rx
        dy = target_wp.pose.position.y - ry
        angle_to_target = math.atan2(dy, dx)
        alpha = math.atan2(
            math.sin(angle_to_target - robot_yaw),
            math.cos(angle_to_target - robot_yaw)
        )   # normalizado a [-π, π]

        # --- 4. Calcular ω con la fórmula de Pure Pursuit ---
        # ω = (2 * v * sin(α)) / L
        L = self.LOOKAHEAD_DISTANCE
        v = self.LINEAR_SPEED
        omega = (2.0 * v * math.sin(alpha)) / L

        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = omega
        return cmd

    def _execute_avoidance_maneuver(self) -> bool:
        """TODO: lógica de evasión local. Retorna True cuando terminó. (Tpf0)"""
        return False

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
        """TODO: verificar si la covarianza creció demasiado durante el recorrido."""
        return False
    
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