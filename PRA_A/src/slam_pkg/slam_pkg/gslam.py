"""[BORRADOR DE REFERENCIA — NO EJECUTABLE] GraphSLAM en forma de información (Thrun).

⚠️  Este archivo es el primer intento de GraphSLAM, en la forma de matriz de
    información online del libro de Thrun (linearize → reduce → solve). Se conserva
    SOLO como referencia conceptual. NO está integrado ni se ejecuta:
      - tiene imports faltantes (`Node`, `Odometry`, un mensaje `ArucoDetection`
        que no existe),
      - no es un entry point en setup.py,
      - espera observaciones rango-bearing-signature con ruido hard-codeado.

    La implementación REAL y usada en la entrega es `graph_slam.py` (mínimos cuadrados
    batch, disperso, con Levenberg-Marquardt y keyframes). Para entender o correr el
    SLAM, mirá ESE archivo, no este.
"""
import numpy as np
import rclpy
import copy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

ODOM_X_ERR = 0.1
ODOM_Y_ERR = 0.1
ODOM_THETA_ERR = 0.05

R_ERR = 0.5
PHI_ERR = 0.1
S_ERR = 1.0

class GraphSLAMNode(Node):
    def __init__(self):
        super().__init__('graph_slam_node')
        
        #  ESTADO DEL ALGORITMO =========================================================
        
        # mu_0:t — vector de medias de poses [x0, y0, θ0, x1, y1, θ1, ...]
        self.mu = [np.array([0.0, 0.0, 0.0])]  # pose inicial es en el origen
        
        # Landmarks conocidos: {marker_id: np.array([x, y])}
        self.landmarks = {}
        
        # Historial de controles: lista de (v, omega, delta_t)
        self.controls = []
        
        # Historial de observaciones: lista de listas:
        # observations[t] = [(marker_id, r, phi, s), ...]
        self.observations = []
        
        # Matrices de información (se construyen en linearize)
        self.Omega = None
        self.xi = None
        
        # Pose actual (para integrar odometría entre optimizaciones)
        self.last_odom = None
        self.current_time = None
        
        # === SUSCRIPCIONES Y PUBLISHERS ===
        self.sub_odom = self.create_subscription(
            Odometry, 'tb4_0/odom', self.odom_callback, 10)
        
        self.sub_aruco = self.create_subscription(
            ArucoDetection, '/aruco_detection', self.aruco_callback, 10)
        
        self.pub_belief    = self.create_publisher(Path,        '/belief',    10)
        self.pub_landmarks = self.create_publisher(MarkerArray, '/landmarks', 10)
        
        # Timer para correr optimización periódicamente *** hacerlo cada ciertas llamadas
        self.create_timer(1.0, self.run_graphslam)
        
        # OBSERVACIONES DEL TIMESTEP ACTUAL ==============================
        self.current_observations = []

    def odom_callback(self, msg):
        # Cada vez que llega odometría nueva, si hay suficiente Δt,
        # "cerramos" el timestep anterior y abrimos uno nuevo
        
        if self.last_odom is None:
            self.last_odom = msg
            return
        
        # Extraer v, omega del mensaje
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        
        dt = (rclpy.time.Time.from_msg(msg.header.stamp) -
            rclpy.time.Time.from_msg(self.last_odom.header.stamp)).nanoseconds / 1e9
        
        if dt < 0.05:  # filtrar timesteps demasiado pequeños
            return
        
        # Guardar control u_t
        self.controls.append((v, omega, dt))
        
        # Las observaciones acumuladas hasta ahora corresponden a este timestep
        self.observations.append(list(self.current_observations))
        self.current_observations = []
        
        # Propagar pose para tener mu inicial (GraphSLAM_initialize)
        new_pose = self.motion_model(self.mu[-1], v, omega, dt)
        self.mu.append(new_pose)
        
        self.last_odom = msg


    def aruco_callback(self, msg):
        # Acumular observaciones hasta el próximo timestep de odometría
        self.current_observations.append(
            (msg.marker_id, msg.range, msg.bearing, msg.signature)
        )
        
        # Inicializar landmark si es nuevo (necesario para linearize)
        if msg.marker_id not in self.landmarks:
            pose = self.mu[-1]
            lx = pose[0] + msg.range * np.cos(pose[2] + msg.bearing)
            ly = pose[1] + msg.range * np.sin(pose[2] + msg.bearing)
            self.landmarks[msg.marker_id] = np.array([lx, ly])
    
    def motion_model(self, pose, v, omega, dt):
            x, y, theta = pose
            
            if abs(omega) < 1e-6:
                # Caso especial: movimiento recto
                # límite cuando ω→0 de las fórmulas anteriores
                x_new = x + v * np.cos(theta) * dt
                y_new = y + v * np.sin(theta) * dt
                theta_new = theta
            else:
                # Caso general del libro
                r = v / omega  # radio de curvatura
                x_new     = x - r * np.sin(theta) + r * np.sin(theta + omega * dt)
                y_new     = y + r * np.cos(theta) - r * np.cos(theta + omega * dt)
                theta_new = theta + omega * dt
            
            # Normalizar ángulo a [-π, π]
            theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))
            
            return np.array([x_new, y_new, theta_new])
    
    def pose_index(self, t):
        # pose t ocupa 3 filas/columnas: 3t, 3t+1, 3t+2
        return 3 * t

    def landmark_index(self, j):
        # los landmarks empiezan después de todas las poses
        T = len(self.mu)  # cantidad de poses
        landmark_list = sorted(self.landmarks.keys())
        j_pos = landmark_list.index(j)
        return 3 * T + 2 * j_pos
    
    def graphslam_linearize(self):
        # Inicialización de las matrices y vectores de información:
        T = len(self.mu) # número de poses
        L = len(self.landmarks) # número de landmarks
        dim = 3 * T + 2 * L # por cada pose tenemos (x,y,theta) y por cada landmark (r, orientación)

        Omega = np.zeros((dim, dim))
        xi = np.zeros(dim)

        # ── Línea 3: anclar la pose inicial con información infinita ──
        # En práctica usamos un número grande en lugar de infinito
        idx0 = self.pose_index(0)
        Omega[idx0:idx0+3, idx0:idx0+3] += np.eye(3) * 1e6

        # ── Restricciones de movimiento ───────────────────
        for t, (v, omega, dt) in enumerate(self.controls):
            mu_prev = self.mu[t]
            x, y, theta = mu_prev

            # x_hat_t: predicción de pose t+1 desde pose t (línea 5)
            x_hat = self.motion_model(mu_prev, v, omega, dt)

            # G_t: jacobiano del modelo de movimiento (línea 6)
            if abs(omega) < 1e-6:
                G_t = np.array([
                    [1, 0, -v * np.sin(theta) * dt],
                    [0, 1,  v * np.cos(theta) * dt],
                    [0, 0,  1]
                ])
            else:
                r = v / omega
                G_t = np.array([
                    [1, 0, -r*np.cos(theta) + r*np.cos(theta + omega*dt)],
                    [0, 1, -r*np.sin(theta) + r*np.sin(theta + omega*dt)],
                    [0, 0,  1]
                ])

            # Matriz de covarianza del ruido de movimiento R_t
            # (cuánto confiás en la odometría)
            R_t = np.diag([ODOM_X_ERR**2, ODOM_Y_ERR**2, ODOM_THETA_ERR**2])
            R_t_inv = np.linalg.inv(R_t)

            # Índices de x_{t} y x_{t+1} en Omega
            i_prev = self.pose_index(t)
            i_curr = self.pose_index(t + 1)

            # Bloque [-G_t^T, 1]^T R_t^-1 [-G_t, 1] (línea 7)
            # Se agrega en los bloques (t,t), (t,t+1), (t+1,t), (t+1,t+1)
            F = np.block([[-G_t], [np.eye(3)]])  # shape (6,3)
            contrib = F @ R_t_inv @ F.T          # shape (6,6)

            Omega[i_prev:i_prev+3, i_prev:i_prev+3] += contrib[0:3, 0:3]
            Omega[i_prev:i_prev+3, i_curr:i_curr+3] += contrib[0:3, 3:6]
            Omega[i_curr:i_curr+3, i_prev:i_prev+3] += contrib[3:6, 0:3]
            Omega[i_curr:i_curr+3, i_curr:i_curr+3] += contrib[3:6, 3:6]

            # Vector xi (línea 8)
            innovation = x_hat - G_t @ mu_prev   # [x_hat - G_t * mu_{t-1}]
            xi_contrib = F @ R_t_inv @ innovation # shape (6,)

            xi[i_prev:i_prev+3] += xi_contrib[0:3]
            xi[i_curr:i_curr+3] += xi_contrib[3:6]

        # ── Líneas 10-21: restricciones de observación ────────────────

        for t, obs_list in enumerate(self.observations):
            for (marker_id, r_meas, phi_meas, s_meas) in obs_list:

                j = marker_id
                mu_t = self.mu[t]
                mu_j = self.landmarks[j]

                # delta (línea 14)
                delta = np.array([mu_j[0] - mu_t[0],
                                mu_j[1] - mu_t[1]])
                q = delta @ delta  # línea 15

                # z_hat: observación esperada (línea 16)
                z_hat = np.array([
                    np.sqrt(q),
                    np.arctan2(delta[1], delta[0]) - mu_t[2],
                    1.0  # firma constante (usamos s=1 siempre)
                ])
                z_hat[1] = np.arctan2(np.sin(z_hat[1]), np.cos(z_hat[1]))

                # z_t medida
                z_t = np.array([r_meas, phi_meas, s_meas])

                # H_t^i: jacobiano de la observación (línea 17)
                sq = np.sqrt(q)
                H_t = (1/q) * np.array([
                    [-sq*delta[0], -sq*delta[1],  0,  sq*delta[0],  sq*delta[1], 0],
                    [ delta[1],    -delta[0],     -q, -delta[1],     delta[0],   0],
                    [ 0,            0,             0,  0,             0,          q]
                ])

                # Matriz de covarianza del ruido de observación Q_t
                Q_t = np.diag([R_ERR**2, PHI_ERR**2, S_ERR**2]) 
                Q_t_inv = np.linalg.inv(Q_t)

                # Índices en Omega
                i_t = self.pose_index(t)
                i_j = self.landmark_index(j)

                # Índices locales dentro del bloque 6D [x_t, m_j]
                # H_t opera sobre [x_t (3D), m_j (2D)] → ignoramos col de firma
                H_t_reduced = H_t[:, :5]  # quitamos última columna (firma)
                # shape: (3, 5) → filas: [r, phi, s], cols: [xt,yt,θt, xj,yj]

                # Agregar H^T Q^-1 H a Omega (línea 18)
                contrib_omega = H_t_reduced.T @ Q_t_inv @ H_t_reduced  # (5,5)

                Omega[i_t:i_t+3, i_t:i_t+3] += contrib_omega[0:3, 0:3]
                Omega[i_t:i_t+3, i_j:i_j+2] += contrib_omega[0:3, 3:5]
                Omega[i_j:i_j+2, i_t:i_t+3] += contrib_omega[3:5, 0:3]
                Omega[i_j:i_j+2, i_j:i_j+2] += contrib_omega[3:5, 3:5]

                # Agregar a xi (línea 19)
                mu_local = np.array([mu_t[0], mu_t[1], mu_t[2],
                                    mu_j[0], mu_j[1]])
                innovation = z_t - z_hat + H_t_reduced @ mu_local
                # normalizar ángulo en la innovación
                innovation[1] = np.arctan2(np.sin(innovation[1]),
                                        np.cos(innovation[1]))
                xi_contrib = H_t_reduced.T @ Q_t_inv @ innovation  # (5,)

                xi[i_t:i_t+3] += xi_contrib[0:3]
                xi[i_j:i_j+2] += xi_contrib[3:5]

        return Omega, xi
    
    def graphslam_reduce(self, Omega, xi): # ***
        Omega_tilde = Omega.copy()
        xi_tilde = xi.copy()

        landmark_list = sorted(self.landmarks.keys())

        for j in landmark_list:
            i_j = self.landmark_index(j)  # índice inicio del landmark en Omega

            # τ(j): conjunto de poses desde las que se observó j
            # Lo reconstruimos buscando qué poses tienen bloque no nulo con j
            tau_j = []
            T = len(self.mu)
            for t in range(T):
                i_t = self.pose_index(t)
                block = Omega_tilde[i_t:i_t+3, i_j:i_j+2]
                if np.any(np.abs(block) > 1e-10):
                    tau_j.append(t)

            if not tau_j:
                continue

            # Ω̃_{jj}: bloque 2x2 del landmark j consigo mismo
            Omega_jj = Omega_tilde[i_j:i_j+2, i_j:i_j+2]
            Omega_jj_inv = np.linalg.inv(Omega_jj)

            # ξ_j: bloque del vector xi correspondiente al landmark j
            xi_j = xi_tilde[i_j:i_j+2]

            # Recopilar índices de todas las poses en τ(j)
            tau_indices = []
            for t in tau_j:
                i_t = self.pose_index(t)
                tau_indices.extend(range(i_t, i_t + 3))

            # Ω̃_{τ(j), j}: bloque rectangular poses × landmark
            Omega_tau_j = Omega_tilde[np.ix_(tau_indices, range(i_j, i_j+2))]

            # Ω̃_{j, τ(j)}: su transpuesta
            Omega_j_tau = Omega_tau_j.T

            # ── Línea 6: actualizar ξ en las poses de τ(j) ──
            # ξ̃_{τ(j)} -= Ω̃_{τ(j),j} · Ω̃_{jj}^{-1} · ξ_j
            xi_tilde[tau_indices] -= Omega_tau_j @ Omega_jj_inv @ xi_j

            # ── Línea 7: actualizar Ω en el bloque poses × poses ──
            # Ω̃_{τ(j),τ(j)} -= Ω̃_{τ(j),j} · Ω̃_{jj}^{-1} · Ω̃_{j,τ(j)}
            Omega_tilde[np.ix_(tau_indices, tau_indices)] -= (
                Omega_tau_j @ Omega_jj_inv @ Omega_j_tau
            )

            # ── Línea 8: eliminar filas/columnas del landmark j ──
            # No las borramos físicamente (complicaría los índices),
            # las zeroeamos para que no afecten el solve
            Omega_tilde[i_j:i_j+2, :] = 0
            Omega_tilde[:, i_j:i_j+2] = 0
            xi_tilde[i_j:i_j+2]       = 0

        return Omega_tilde, xi_tilde
    
    def graphslam_solve(self, Omega_tilde, xi_tilde, Omega, xi): # ***
        T = len(self.mu)
        landmark_list = sorted(self.landmarks.keys())

        # ── Líneas 2-3: resolver el sistema de poses ──────────────────
        # Extraer solo el bloque de poses de Ω̃ (ignorar filas/cols de landmarks)
        pose_dim = 3 * T
        Omega_poses = Omega_tilde[:pose_dim, :pose_dim]
        xi_poses    = xi_tilde[:pose_dim]

        # Σ = Ω̃^{-1},  μ_{0:t} = Σ · ξ̃
        # Usar solve es más estable numéricamente que invertir directamente
        Sigma_poses = np.linalg.inv(Omega_poses)
        mu_poses    = Sigma_poses @ xi_poses # PATH ESTIMATES !!!

        # Actualizar self.mu con las poses optimizadas
        for t in range(T):
            i_t = self.pose_index(t)
            self.mu[t] = mu_poses[i_t:i_t+3]
            # Normalizar ángulo
            self.mu[t][2] = np.arctan2(np.sin(self.mu[t][2]),
                                    np.cos(self.mu[t][2]))

        # ── Líneas 4-7: recuperar posiciones de landmarks ─────────────
        for j in landmark_list:
            i_j = self.landmark_index(j)

            # τ(j): poses desde las que se observó j
            # Buscamos en Omega ORIGINAL (no en Omega_tilde, que fue zeroeada)
            tau_j = []
            for t in range(T):
                i_t = self.pose_index(t)
                block = Omega[i_t:i_t+3, i_j:i_j+2]
                if np.any(np.abs(block) > 1e-10):
                    tau_j.append(t)

            if not tau_j:
                continue

            tau_indices = []
            for t in tau_j:
                i_t = self.pose_index(t)
                tau_indices.extend(range(i_t, i_t + 3))

            # Ω_{jj} y ξ_j de la Omega ORIGINAL
            Omega_jj     = Omega[i_j:i_j+2, i_j:i_j+2]
            Omega_jj_inv = np.linalg.inv(Omega_jj)
            xi_j         = xi[i_j:i_j+2]

            # Ω_{j, τ(j)}: bloque landmark × poses
            Omega_j_tau = Omega[np.ix_(range(i_j, i_j+2), tau_indices)]

            # μ_{τ(j)}: poses ya optimizadas
            mu_tau = mu_poses[tau_indices]

            # μ_j = Ω_{jj}^{-1} · (ξ_j + Ω_{j,τ(j)} · μ̃_{τ(j)})  (línea 6)
            self.landmarks[j] = Omega_jj_inv @ (xi_j + Omega_j_tau @ mu_tau)

        return self.mu, self.landmarks, Sigma_poses
    
    def run_graphslam(self):
        if len(self.controls) < 2 or len(self.landmarks) == 0:
            return  # no hay suficientes datos todavía

        self.Omega, self.xi = self.graphslam_linearize()
        Omega_tilde, xi_tilde = self.graphslam_reduce(self.Omega, self.xi)
        mu_poses, mu_landmarks, Sigma_landmarks = self.graphslam_solve(Omega_tilde, xi_tilde,
                                                    self.Omega, self.xi)

        self.publish_belief()
        self.publish_landmarks()
