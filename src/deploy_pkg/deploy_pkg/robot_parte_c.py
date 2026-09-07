"""
Parte C — Nodo de navegación autónoma para el TurtleBot4 real.

Extiende la FSM de Parte B (RobotNavigator) con tres estados nuevos:
  EXPLORING      — recorre el laberinto reactivamente buscando conos rojos
  CONE_DETECTED  — convierte la coord. relativa del cono al frame del mapa
                   y lanza la planificación hacia él (con offset para no
                   apuntar a la pared donde está apoyado el cono)

Cambios respecto a Parte B:
  - Topics remapeados a /tb4_0/* (robot real, no Gazebo TB3)
  - QoS BEST_EFFORT en /tb4_0/scan y /tb4_0/odom
  - Nueva suscripción a /tb4_0/cono_detectado (PointStamped del detector)
  - APPROACHING_CONE eliminado: el planificador Theta* maneja todo el
    camino hasta el goal offset; ALIGNING hace el ajuste final.
    El P-controller sin mapa causaba que el robot atravesara paredes.
"""

import math
import rclpy
from enum import Enum, auto
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from .robot import RobotNavigator, State


class StateC(Enum):
    WAITING       = auto()
    EXPLORING     = auto()
    CONE_DETECTED = auto()
    PLANNING      = auto()
    WALKING       = auto()
    AVOIDING      = auto()
    ALIGNING      = auto()


class RobotNavigatorC(RobotNavigator):

    def __init__(self):
        super().__init__()
        self.get_logger().info('=== Parte C: modo TurtleBot4 real ===')

        self.timer.cancel()

        # --- Estado inicial ---
        self.state_c = StateC.WAITING

        # --- Exploración ---
        self.EXPLORE_LINEAR_SPEED    = 0.12
        self.EXPLORE_ANGULAR_SPEED   = 0.5
        self.EXPLORE_OBSTACLE_DIST   = 0.4
        self.EXPLORE_CONE_HALF_ANGLE = math.radians(40)
        self._explore_turning        = False
        self._explore_turn_sign      = 1.0

        # --- Cono ---
        # El goal se pone a CONE_GOAL_OFFSET metros ANTES del cono para que
        # caiga en espacio libre y no dentro de la pared donde está apoyado.
        self.CONE_GOAL_OFFSET = 0.40   # metros
        self.cone_point: PointStamped | None = None
        self.cone_goal:  PoseStamped  | None = None

        self._remap_topics_for_tb4()
        self.timer_c = self.create_timer(0.1, self.state_machine_loop_c)

    def _remap_topics_for_tb4(self):
        """
        Remaps the topics to /tb4_0/* for the real TurtleBot4, and sets QoS for LaserScan.
        """
        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.destroy_subscription(self.sub_scan)
        self.sub_scan = self.create_subscription(
            LaserScan, '/tb4_0/scan', self.cb_scan, qos_be)

        self.destroy_publisher(self.pub_cmd_vel)
        self.pub_cmd_vel = self.create_publisher(Twist, '/tb4_0/cmd_vel', 10)

        self.sub_cono = self.create_subscription(
            PointStamped, '/tb4_0/cono_detectado', self.cb_cono, 10)

        self.get_logger().info('Topics remapeados a /tb4_0/*')

    def cb_cono(self, msg: PointStamped):
        """
        Callback function for the /tb4_0/cono_detectado topic. 
        It stores the detected cone point and transitions to CONE_DETECTED state if appropriate.
        """
        if self.state_c not in (StateC.EXPLORING, StateC.WAITING):
            return
        self.cone_point = msg
        self.get_logger().info(
            f'Cono detectado en robot-frame: '
            f'x={msg.point.x:.2f} y={msg.point.y:.2f}')
        self._transition_c(StateC.CONE_DETECTED)

    def state_machine_loop_c(self):
        """
        Main loop for the extended state machine (FSM-C). 
        It checks localization and executes the appropriate state handler based on the current state.
        """
        if not self._localization_ok():
            self.get_logger().warn(
                'Localización no confiable. Robot detenido.',
                throttle_duration_sec=2.0)
            return

        match self.state_c:
            case StateC.WAITING:
                self._run_waiting_c()
            case StateC.EXPLORING:
                self._run_exploring()
            case StateC.CONE_DETECTED:
                self._run_cone_detected()
            case StateC.PLANNING:
                self.new_goal_received = False
                self.run_planning()
                self._sync_state_c_from_b()
            case StateC.WALKING:
                self._run_walking_c()
            case StateC.AVOIDING:
                self.run_avoiding()
                self._sync_state_c_from_b()
            case StateC.ALIGNING:
                self._run_aligning_c()

    def _run_waiting_c(self):
        """
        Waits for localization to converge. Once converged, transitions to EXPLORING state.
        """
        if self._check_localization_converged('waiting_c'):
            self.get_logger().info('Localización lista. Iniciando exploración.')
            self._transition_c(StateC.EXPLORING)

    def _run_exploring(self):
        """
        Reactive navigation to explore the maze and look for red cones. 
        If the front is clear, it moves forward; otherwise, it turns to avoid obstacles.
        """
        if self.last_scan is None:
            return

        if self._explore_front_clear():
            self._explore_turning = False
            cmd = Twist()
            cmd.linear.x = self.EXPLORE_LINEAR_SPEED
            self.pub_cmd_vel.publish(cmd)
        else:
            if not self._explore_turning:
                self._explore_turn_sign = self._pick_clearer_side()
                self._explore_turning = True
                lado = 'izquierda' if self._explore_turn_sign > 0 else 'derecha'
                self.get_logger().info(
                    f'Exploración: obstáculo, girando {lado}.',
                    throttle_duration_sec=1.0)
            cmd = Twist()
            cmd.angular.z = self._explore_turn_sign * self.EXPLORE_ANGULAR_SPEED
            self.pub_cmd_vel.publish(cmd)

    def _run_cone_detected(self):
        """
        Converts the detected cone position from the robot frame to the map frame and sets the goal
        to be CONE_GOAL_OFFSET meters before the cone (in free space, not at the wall).
        """
        if self.cone_point is None or self.current_pose is None:
            self.get_logger().warn('CONE_DETECTED: faltan datos, volviendo a explorar.')
            self._transition_c(StateC.EXPLORING)
            return

        robot_x = self.current_pose.pose.position.x
        robot_y = self.current_pose.pose.position.y
        yaw     = self.current_yaw

        cx = self.cone_point.point.x
        cy = self.cone_point.point.y
        dist = math.hypot(cx, cy)

        # Retroceder CONE_GOAL_OFFSET metros desde el cono hacia el robot
        if dist > self.CONE_GOAL_OFFSET:
            factor = (dist - self.CONE_GOAL_OFFSET) / dist
            cx_adj = cx * factor
            cy_adj = cy * factor
        else:
            cx_adj, cy_adj = 0.0, 0.0

        # Rotar al frame del mapa
        cone_map_x = robot_x + cx_adj * math.cos(yaw) - cy_adj * math.sin(yaw)
        cone_map_y = robot_y + cx_adj * math.sin(yaw) + cy_adj * math.cos(yaw)

        self.get_logger().info(
            f'Goal cono en mapa: ({cone_map_x:.2f}, {cone_map_y:.2f}) '
            f'[dist real={dist:.2f}m, offset={self.CONE_GOAL_OFFSET}m]')

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = cone_map_x
        goal.pose.position.y = cone_map_y
        goal.pose.orientation.w = 1.0

        self.goal_pose = goal
        self.cone_goal = goal
        self.new_goal_received = False
        self.state = State.PLANNING
        self._transition_c(StateC.PLANNING)

    def _run_walking_c(self):
        """
        Walking towards the goal. If a new goal is received, an obstacle is detected, or the goal is reached,
        it transitions to the appropriate state.
        """
        if self.new_goal_received:
            self.new_goal_received = False
            self._stop_robot()
            self.state = State.PLANNING
            self._transition_c(StateC.PLANNING)
            return

        if self._obstacle_detected_on_path():
            self._stop_robot()
            self.state = State.AVOIDING
            self._transition_c(StateC.AVOIDING)
            return

        if self._reached_goal_position():
            self._stop_robot()
            self.state = State.ALIGNING
            self._transition_c(StateC.ALIGNING)
            return

        cmd = self._compute_pure_pursuit_cmd()
        self.pub_cmd_vel.publish(cmd)

    def _run_aligning_c(self):
        """
        Final alignment to the goal orientation after reaching the goal position. 
        Once aligned, it stops the robot and transitions back to EXPLORING state.
        """
        self.state = State.ALIGNING
        done = self._align_to_goal_angle()
        if done:
            self._stop_robot()
            self.cone_goal  = None
            self.cone_point = None
            self.get_logger().info('Cono alcanzado. Volviendo a explorar.')
            self._transition_c(StateC.EXPLORING)

    def _explore_front_clear(self) -> bool:
        """
        Checks if the front of the robot is clear of obstacles within the EXPLORE_CONE_HALF_ANGLE and EXPLORE_OBSTACLE_DIST.
        Returns True if clear, False if an obstacle is detected.
        """
        msg = self.last_scan
        if msg is None:
            return True
        angle = msg.angle_min
        for r in msg.ranges:
            a = math.atan2(math.sin(angle), math.cos(angle))
            if abs(a) <= self.EXPLORE_CONE_HALF_ANGLE:
                if (math.isfinite(r)
                        and msg.range_min < r < msg.range_max
                        and r < self.EXPLORE_OBSTACLE_DIST):
                    return False
            angle += msg.angle_increment
        return True

    def _transition_c(self, new_state: StateC):
        """
        Transitions to a new state in the extended FSM-C and logs the transition.
        """
        self.get_logger().info(f'[FSM-C] {self.state_c.name} → {new_state.name}')
        self.state_c = new_state

    def _sync_state_c_from_b(self):
        """
        Synchronizes the extended FSM-C state with the base FSM-B state.
        """
        mapping = {
            State.WAITING:  StateC.WAITING,
            State.PLANNING: StateC.PLANNING,
            State.WALKING:  StateC.WALKING,
            State.AVOIDING: StateC.AVOIDING,
            State.ALIGNING: StateC.ALIGNING,
        }
        if self.state in mapping:
            target = mapping[self.state]
            if target != self.state_c:
                self._transition_c(target)

    def run_planning(self):
        """
        Runs the planning logic for the extended FSM-C.
        """
        super().run_planning()
        if self.state == State.WALKING and self.state_c == StateC.PLANNING:
            self._transition_c(StateC.WALKING)
        elif self.state == State.WAITING and self.state_c == StateC.PLANNING:
            self.cone_goal  = None
            self.cone_point = None
            self._transition_c(StateC.EXPLORING)
            

def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigatorC()
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
