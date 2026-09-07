"""GraphSLAM 2D por mínimos cuadrados (Gauss-Newton disperso).

Parte A — Opción 3 (Features con Cámara). Optimiza una trayectoria de poses
(x, y, θ) y un conjunto de landmarks (lx, ly) a partir de:

  - Restricciones de movimiento (odometría): medición relativa entre dos poses
    consecutivas, construida sobre los deltas (δrot1, δtrans, δrot2) que ya
    calcula `odom_delta_node`.
  - Restricciones de observación (ArUco): medición rango-bearing de un landmark
    visto desde una pose, con covarianza dependiente de la distancia.

Diferencia con `gslam.py` (borrador previo, forma de información online estilo
Thrun): acá resolvemos el problema **batch** por mínimos cuadrados no lineales.
Es matemáticamente equivalente (la matriz de información Ω = JᵀΩ_zJ), pero:
  - Itera (re-lineariza) → maneja bien la no linealidad de los ángulos.
  - Usa álgebra dispersa (scipy.sparse) + keyframes → escala a las ~28k lecturas
    del laberinto, donde una matriz densa sería inviable.
  - No depende de g2o/GTSAM → el paquete queda autocontenido (lo pide la consigna).

El cierre de lazo (loop closure) sale "gratis" cuando una pose tardía vuelve a
observar un landmark ArUco ya visto: esa observación ata el grafo y corrige la
deriva. La asociación de datos es trivial porque cada tag trae su ID.

Ejecutar el autotest sintético:
    python3 graph_slam.py
"""

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve


# --------------------------------------------------------------------------- #
# Utilidades SE(2)
# --------------------------------------------------------------------------- #
def wrap(a):
    """Normaliza un ángulo (o array) a [-π, π]."""
    return np.arctan2(np.sin(a), np.cos(a))


def relative_pose(pi, pj):
    """Movimiento de la pose i a la pose j, expresado en el frame de i.

    Devuelve (dx, dy, dθ). Es la 'medición' que predice una restricción de
    odometría; también se usa para fabricar mediciones sintéticas.
    """
    c, s = np.cos(pi[2]), np.sin(pi[2])
    dx, dy = pj[0] - pi[0], pj[1] - pi[1]
    return np.array([c * dx + s * dy,
                     -s * dx + c * dy,
                     wrap(pj[2] - pi[2])])


# --------------------------------------------------------------------------- #
# Estructura del problema
# --------------------------------------------------------------------------- #
class GraphSLAM:
    """Grafo de poses + landmarks y su optimización por Gauss-Newton.

    Layout del vector de estado x:
        [x0,y0,θ0, x1,y1,θ1, ..., lx0,ly0, lx1,ly1, ...]
    Las poses ocupan 3 componentes; los landmarks 2.
    """

    def __init__(self):
        self.poses = []                 # lista de np.array([x, y, θ])  (estimación inicial)
        self.landmark_ids = []          # ids ArUco en orden de inserción
        self.landmarks = {}             # id -> np.array([lx, ly])
        self.odom_edges = []            # (i, j, z(3,), info(3,3))
        self.obs_edges = []             # (i, lid, z(2,)=[range,bearing], info(2,2))
        self.pose_times = None          # timestamp de cada pose/keyframe (para asociar scans)

    # -- construcción -------------------------------------------------------- #
    def add_pose(self, pose):
        self.poses.append(np.asarray(pose, dtype=float))
        return len(self.poses) - 1

    def add_landmark(self, lid, xy):
        if lid not in self.landmarks:
            self.landmarks[lid] = np.asarray(xy, dtype=float)
            self.landmark_ids.append(lid)

    def add_odom_edge(self, i, j, z, info):
        self.odom_edges.append((i, j, np.asarray(z, float), np.asarray(info, float)))

    def add_obs_edge(self, i, lid, z, info):
        self.obs_edges.append((i, lid, np.asarray(z, float), np.asarray(info, float)))

    # -- indexado ------------------------------------------------------------ #
    @property
    def n_poses(self):
        return len(self.poses)

    def _pose_idx(self, i):
        return 3 * i

    def _lm_idx(self, lid):
        return 3 * self.n_poses + 2 * self.landmark_ids.index(lid)

    @property
    def dim(self):
        return 3 * self.n_poses + 2 * len(self.landmark_ids)

    # -- residuos y jacobianos ---------------------------------------------- #
    @staticmethod
    def _motion_residual_jac(pi, pj, z):
        """Restricción de movimiento. r = pred(pi,pj) - z, y sus jacobianos.

        pred = pose relativa de j vista desde i (ver relative_pose()).
        """
        c, s = np.cos(pi[2]), np.sin(pi[2])
        dx, dy = pj[0] - pi[0], pj[1] - pi[1]
        pred = np.array([c * dx + s * dy,
                         -s * dx + c * dy,
                         wrap(pj[2] - pi[2])])
        r = np.array([pred[0] - z[0], pred[1] - z[1], wrap(pred[2] - z[2])])

        Ji = np.array([
            [-c, -s, -s * dx + c * dy],
            [s, -c, -c * dx - s * dy],
            [0.0, 0.0, -1.0],
        ])
        Jj = np.array([
            [c, s, 0.0],
            [-s, c, 0.0],
            [0.0, 0.0, 1.0],
        ])
        return r, Ji, Jj

    @staticmethod
    def _obs_residual_jac(pi, lk, z):
        """Restricción de observación rango-bearing. r = pred - z, jacobianos."""
        dx, dy = lk[0] - pi[0], lk[1] - pi[1]
        q = dx * dx + dy * dy
        rng = np.sqrt(q)
        pred = np.array([rng, wrap(np.arctan2(dy, dx) - pi[2])])
        r = np.array([pred[0] - z[0], wrap(pred[1] - z[1])])

        # Jacobiano respecto de la pose (x, y, θ)
        Jp = np.array([
            [-dx / rng, -dy / rng, 0.0],
            [dy / q, -dx / q, -1.0],
        ])
        # Jacobiano respecto del landmark (lx, ly)
        Jl = np.array([
            [dx / rng, dy / rng],
            [-dy / q, dx / q],
        ])
        return r, Jp, Jl

    # -- optimización -------------------------------------------------------- #
    def _state_vector(self):
        x = np.zeros(self.dim)
        for i, p in enumerate(self.poses):
            x[3 * i:3 * i + 3] = p
        for lid in self.landmark_ids:
            k = self._lm_idx(lid)
            x[k:k + 2] = self.landmarks[lid]
        return x

    def _write_state(self, x):
        for i in range(self.n_poses):
            self.poses[i] = x[3 * i:3 * i + 3].copy()
            self.poses[i][2] = wrap(self.poses[i][2])
        for lid in self.landmark_ids:
            k = self._lm_idx(lid)
            self.landmarks[lid] = x[k:k + 2].copy()

    def _compute_chi2(self):
        """Error ponderado total (sin el ancla) en el estado actual."""
        chi2 = 0.0
        for (i, j, z, info) in self.odom_edges:
            r, _, _ = self._motion_residual_jac(self.poses[i], self.poses[j], z)
            chi2 += r @ info @ r
        for (i, lid, z, info) in self.obs_edges:
            r, _, _ = self._obs_residual_jac(self.poses[i], self.landmarks[lid], z)
            chi2 += r @ info @ r
        return chi2

    def _linearize(self, anchor_info):
        """Arma H (csr), b y chi2 en el estado actual.
            H = Σ Jᵀ Ω J + ancla,   b = Σ Jᵀ Ω r
        """
        rows, cols, vals = [], [], []
        b = np.zeros(self.dim)

        def add_block(idx_a, idx_b, Ja, Jb, info, r):
            blocks = [(idx_a, Ja), (idx_b, Jb)]
            for (ia, Ma) in blocks:
                b[ia:ia + Ma.shape[1]] += Ma.T @ info @ r
                for (ib, Mb) in blocks:
                    Hab = Ma.T @ info @ Mb
                    for p in range(Hab.shape[0]):
                        for qd in range(Hab.shape[1]):
                            rows.append(ia + p); cols.append(ib + qd); vals.append(Hab[p, qd])

        # Ancla de la pose 0 (fija el gauge global).
        i0 = self._pose_idx(0)
        for d in range(3):
            rows.append(i0 + d); cols.append(i0 + d); vals.append(anchor_info)

        chi2 = 0.0
        for (i, j, z, info) in self.odom_edges:
            r, Ji, Jj = self._motion_residual_jac(self.poses[i], self.poses[j], z)
            chi2 += r @ info @ r
            add_block(self._pose_idx(i), self._pose_idx(j), Ji, Jj, info, r)
        for (i, lid, z, info) in self.obs_edges:
            r, Jp, Jl = self._obs_residual_jac(self.poses[i], self.landmarks[lid], z)
            chi2 += r @ info @ r
            add_block(self._pose_idx(i), self._lm_idx(lid), Jp, Jl, info, r)

        H = coo_matrix((vals, (rows, cols)), shape=(self.dim, self.dim)).tocsr()
        return H, b, chi2

    def optimize(self, iterations=50, anchor_info=1e6, tol=1e-8, verbose=True):
        """Levenberg-Marquardt disperso. Ancla la pose 0 (gauge) y solo acepta
        pasos que reducen el error, ajustando el damping λ. Es robusto frente a la
        divergencia que sufre el Gauss-Newton puro en grafos grandes o mal
        condicionados (donde un paso puede mandar las poses al infinito).
        """
        from scipy.sparse import diags
        history = []
        lam = 1e-3
        for it in range(iterations):
            H, b, chi2 = self._linearize(anchor_info)
            diagH = H.diagonal()
            x0 = self._state_vector()
            accepted, delta, chi2_new = False, None, chi2
            for _ in range(12):
                Hlm = (H + diags(lam * diagH)).tocsc()   # damping escalado por la curvatura
                try:
                    delta = spsolve(Hlm, -b)
                except Exception:
                    delta = None
                if delta is None or not np.all(np.isfinite(delta)):
                    lam *= 10.0
                    continue
                self._write_state(x0 + delta)
                chi2_new = self._compute_chi2()
                if np.isfinite(chi2_new) and chi2_new < chi2:
                    lam = max(lam * 0.5, 1e-12)
                    accepted = True
                    break
                self._write_state(x0)        # rechazar el paso y subir el damping
                lam *= 10.0

            step = float(np.linalg.norm(delta)) if (accepted and delta is not None) else 0.0
            history.append(chi2_new if accepted else chi2)
            if verbose:
                print(f"  iter {it:2d}  chi2={(chi2_new if accepted else chi2):12.4f}  "
                      f"lambda={lam:.1e}  |delta|={step:.3e}")
            if not accepted:
                if verbose:
                    print("  (sin mejora — LM detenido)")
                break
            if step < tol:
                break
        return history

    def gate_observations(self, chi2_thresh=13.8):
        """Gating de outliers (estilo EKF de tp4): descarta los edges de
        observación cuyo residuo de Mahalanobis (rᵀ·Ω·r, con Ω la información de
        la medición) supera el umbral χ²(2 g.l.) en el estado YA optimizado.
        Sirve para sacar detecciones ArUco espurias o mal asociadas. Umbrales
        típicos χ²₂: 5.99 (95%), 9.21 (99%), 13.8 (99.9%, como tp4).

        Devuelve cuántas observaciones se quitaron. También elimina landmarks que
        quedaron sin ninguna observación (si no, su bloque deja H singular).
        """
        kept, removed = [], 0
        for (i, lid, z, info) in self.obs_edges:
            r, _, _ = self._obs_residual_jac(self.poses[i], self.landmarks[lid], z)
            if float(r @ info @ r) <= chi2_thresh:
                kept.append((i, lid, z, info))
            else:
                removed += 1
        self.obs_edges = kept
        used = {lid for (_, lid, _, _) in kept}
        for lid in [l for l in self.landmark_ids if l not in used]:
            del self.landmarks[lid]
            self.landmark_ids.remove(lid)
        return removed


# --------------------------------------------------------------------------- #
# Modelo de ruido del sensor ArUco (de fit_noise_model.py → noise_model.json)
# --------------------------------------------------------------------------- #
def _load_noise_params(path):
    """Lee el noise_model.json de fit_noise_model.py: {axis: {a, b}}, con std=a+b·d
    por eje cartesiano (tx lateral, ty vertical, tz profundidad), en metros."""
    import json
    with open(path) as f:
        return json.load(f)


def _noise_obs_info(rng, bearing, nm, scale_unc, bearing_floor):
    """Matriz de información 2×2 (rango, bearing) de un edge ArUco, a partir del
    modelo de ruido fiteado del sensor.

    El fit caracteriza el ruido en coordenadas de cámara (tx lateral, tz profundidad);
    acá se propaga a (rango, bearing) por el Jacobiano del paso a polares
    (r=√(bx²+by²), φ=atan2(by,bx), con bx≈tz adelante, by≈-tx lateral). Dos detalles
    físicos que hacen que esto mejore el mapa en vez de romperlo:
      - El bearing es INVARIANTE a la escala del marcador, y el fit lo da muy fino →
        conviene confiar en él (ArUco corrige fuerte el RUMBO, clave para el cierre de
        lazo). El valor que estaba hardcodeado (12°) lo subestimaba por mucho.
      - El RANGO sí arrastra el error de `marker_length` (es una estimación, no medido):
        un error de escala ε produce un error de rango ε·r que el modelo de ruido
        *aleatorio* NO captura. Se suma como término sistemático (scale_unc·r)², así el
        rango queda apropiadamente flojo a media/larga distancia y ArUco no deforma el
        mapa. (Meter el modelo fiteado crudo, sin este término, sobre-confía el rango
        ~80× → el gating tira la mitad de las obs y la trayectoria empeora. Verificado.)
    """
    s_tz = max(nm['tz']['a'] + nm['tz']['b'] * rng, 1e-4)     # profundidad → rango
    s_tx = max(nm['tx']['a'] + nm['tx']['b'] * rng, 1e-4)     # lateral → bearing
    bx, by = rng * np.cos(bearing), rng * np.sin(bearing)
    r = max(rng, 1e-3)
    J = np.array([[bx / r, by / r],
                  [-by / r ** 2, bx / r ** 2]])
    cov = J @ np.diag([s_tz ** 2, s_tx ** 2]) @ J.T
    cov[0, 0] += (scale_unc * r) ** 2            # error de escala del marker_length (sistemático)
    cov[1, 1] += bearing_floor ** 2              # piso angular
    return np.linalg.inv(cov)


# --------------------------------------------------------------------------- #
# Carga desde CSV (formato de aruco_pkg)
# --------------------------------------------------------------------------- #
def build_from_csv(odom_csv, aruco_csv=None,
                   kf_trans=0.10, kf_rot=np.deg2rad(10),
                   odom_std=(0.01, 0.01, np.deg2rad(1.0)),    # odom TB4 es muy buena → confiamos fuerte
                   range_std_a=0.40, range_std_b=0.15,        # rango ArUco flojo (fallback sin noise model)
                   bearing_std=np.deg2rad(12),                # bearing flojo (fallback sin noise model)
                   min_detections=10,
                   cam_offset=(-0.0596, 0.0),                 # extrínseca cámara→base (tf_static): ~6cm atrás
                   noise_model_path=None,                     # JSON de fit_noise_model.py; None → std hardcodeado
                   scale_uncertainty=0.08,                    # incertidumbre relativa del marker_length (rango)
                   bearing_floor=np.deg2rad(0.5)):            # piso angular del bearing
    """Construye el grafo desde los CSV reales.

    Submuestrea la odometría a *keyframes* (acumula movimiento hasta superar
    kf_trans [m] o kf_rot [rad]) para que el grafo no tenga decenas de miles de
    nodos. Las observaciones ArUco se asocian al keyframe temporalmente más
    cercano. Requiere pandas.
    """
    import pandas as pd

    # Modelo de ruido fiteado (opcional). Si se pasa, la covarianza de cada
    # observación sale de él (propagada a rango-bearing); si no, se usan los std
    # hardcodeados de abajo (comportamiento previo, no rompe nada).
    nm = _load_noise_params(noise_model_path) if noise_model_path else None

    odom = pd.read_csv(odom_csv).sort_values('timestamp').reset_index(drop=True)
    g = GraphSLAM()

    # Keyframes: primera lectura + cada vez que el movimiento acumulado supera umbral.
    kf_rows = [0]
    acc_t, acc_r = 0.0, 0.0
    for k in range(1, len(odom)):
        acc_t += abs(odom['delta_trans'].iloc[k])
        acc_r += abs(odom['delta_rot1'].iloc[k]) + abs(odom['delta_rot2'].iloc[k])
        if acc_t >= kf_trans or acc_r >= kf_rot:
            kf_rows.append(k)
            acc_t, acc_r = 0.0, 0.0
    if kf_rows[-1] != len(odom) - 1:
        kf_rows.append(len(odom) - 1)

    # Pose inicial de cada keyframe = odometría cruda (luego se corrige).
    for k in kf_rows:
        g.add_pose([odom['x'].iloc[k], odom['y'].iloc[k], odom['theta'].iloc[k]])
    # Timestamps de cada keyframe → permiten asociar cada barrido LIDAR a la pose
    # corregida correspondiente (interpolando) en la 2da pasada.
    g.pose_times = odom['timestamp'].iloc[kf_rows].to_numpy()

    # Edge de odometría entre keyframes: medición relativa desde las poses crudas,
    # con covarianza creciente con la distancia recorrida en el tramo.
    info_step = np.diag(1.0 / np.array(odom_std) ** 2)
    for e in range(len(kf_rows) - 1):
        ka, kb = kf_rows[e], kf_rows[e + 1]
        pa = np.array([odom['x'].iloc[ka], odom['y'].iloc[ka], odom['theta'].iloc[ka]])
        pb = np.array([odom['x'].iloc[kb], odom['y'].iloc[kb], odom['theta'].iloc[kb]])
        z = relative_pose(pa, pb)
        seg = max(1, kb - ka)
        # incertidumbre ~ √(pasos) (no lineal); mantiene la odom fuerte porque es muy buena
        g.add_odom_edge(e, e + 1, z, info_step / np.sqrt(seg))

    # Observaciones ArUco, AGREGADAS por (keyframe, landmark).
    # El robot a ~kf_trans por keyframe ve el mismo tag en muchos frames; meter una
    # observación por frame (decenas de miles) satura el solver y sobre-pondera. En
    # cambio promediamos rango/bearing en UNA observación por par → menos edges,
    # sistema mejor condicionado y mucho más rápido.
    if aruco_csv is not None:
        det = pd.read_csv(aruco_csv)
        counts = det['marker_id'].value_counts()
        good = set(counts[counts >= min_detections].index)  # descarta IDs espurios
        kf_t = odom['timestamp'].iloc[kf_rows].values

        agg = {}   # (kf, lid) -> [sum_range, sum_sin, sum_cos, n]
        for _, row in det.iterrows():
            lid = int(row['marker_id'])
            if lid not in good:
                continue
            # Conversión cámara→base con la extrínseca real del tf_static.
            # Frame óptico OpenCV (x derecha, y abajo, z adelante) → base_link:
            #   x_base = z_opt (adelante),  y_base = -x_opt (izquierda),
            # + traslación de la cámara (oakd está ~6 cm detrás del origen del robot).
            tx, tz = float(row['tx']), float(row['tz'])
            bx = tz + cam_offset[0]
            by = -tx + cam_offset[1]
            rng = float(np.hypot(bx, by))
            bearing = float(np.arctan2(by, bx))
            kf = int(np.argmin(np.abs(kf_t - row['timestamp'])))
            a = agg.setdefault((kf, lid), [0.0, 0.0, 0.0, 0])
            a[0] += rng; a[1] += np.sin(bearing); a[2] += np.cos(bearing); a[3] += 1

        for (kf, lid), (sr, ss, sc, n) in agg.items():
            rng = sr / n
            bearing = float(np.arctan2(ss, sc))   # promedio circular del ángulo
            p = g.poses[kf]
            g.add_landmark(lid, [p[0] + rng * np.cos(p[2] + bearing),
                                 p[1] + rng * np.sin(p[2] + bearing)])
            if nm is not None:
                info = _noise_obs_info(rng, bearing, nm, scale_uncertainty, bearing_floor)
            else:
                r_std = range_std_a + range_std_b * rng    # ruido crece con distancia
                info = np.diag([1.0 / r_std ** 2, 1.0 / bearing_std ** 2])
            g.add_obs_edge(kf, lid, [rng, bearing], info)

    return g


# --------------------------------------------------------------------------- #
# Autotest sintético (verifica el optimizador sin depender de los bags)
# --------------------------------------------------------------------------- #
def _synthetic_selftest(seed=0):
    """Trayectoria cuadrada cerrada (loop) + landmarks. Comprueba que la
    optimización reduce el error frente a la odometría cruda (deriva) y recupera
    la verdad-terreno, demostrando el cierre de lazo."""
    rng = np.random.default_rng(seed)

    # --- verdad-terreno: lazo cuadrado de 4 m de lado, 40 poses ---
    n = 40
    gt = []
    per_side = n // 4
    x, y, th = 0.0, 0.0, 0.0
    step = 4.0 / per_side
    for side in range(4):
        for _ in range(per_side):
            gt.append([x, y, th])
            x += step * np.cos(th)
            y += step * np.sin(th)
        th = wrap(th + np.pi / 2)   # giro de 90° en cada esquina
    gt = np.array(gt[:n])

    # --- landmarks alrededor del recorrido ---
    lms = {0: [2.0, -1.0], 1: [5.0, 2.0], 2: [2.0, 5.0], 3: [-1.0, 2.0]}

    odom_std = np.array([0.04, 0.04, np.deg2rad(2)])
    rstd, bstd = 0.05, np.deg2rad(2)

    g = GraphSLAM()

    # Trayectoria inicial por integración de odometría ruidosa (acumula deriva).
    est = [np.array(gt[0], float)]
    g.add_pose(est[0])
    for i in range(1, n):
        z = relative_pose(gt[i - 1], gt[i]) + rng.normal(0, odom_std)
        # integrar para la estimación inicial
        p = est[-1]
        c, s = np.cos(p[2]), np.sin(p[2])
        nxt = np.array([p[0] + c * z[0] - s * z[1],
                        p[1] + s * z[0] + c * z[1],
                        wrap(p[2] + z[2])])
        est.append(nxt)
        g.add_pose(nxt)
        g.add_odom_edge(i - 1, i, z, np.diag(1.0 / odom_std ** 2))

    odom_only = np.array(est)

    # Observaciones: cada pose ve los landmarks a < 6 m.
    for i in range(n):
        p = gt[i]
        for lid, lxy in lms.items():
            dx, dy = lxy[0] - p[0], lxy[1] - p[1]
            d = np.hypot(dx, dy)
            if d < 6.0:
                z = np.array([d + rng.normal(0, rstd),
                              wrap(np.arctan2(dy, dx) - p[2] + rng.normal(0, bstd))])
                if lid not in g.landmarks:
                    # init landmark desde la pose estimada (ruidosa)
                    pe = g.poses[i]
                    g.add_landmark(lid, [pe[0] + z[0] * np.cos(pe[2] + z[1]),
                                         pe[1] + z[0] * np.sin(pe[2] + z[1])])
                g.add_obs_edge(i, lid, z, np.diag([1.0 / rstd ** 2, 1.0 / bstd ** 2]))

    print("Optimizando grafo sintético (lazo cuadrado, 40 poses, 4 landmarks)...")
    g.optimize(iterations=20)

    opt = np.array(g.poses)

    def rmse(a):
        return float(np.sqrt(np.mean((a[:, :2] - gt[:, :2]) ** 2)))

    e_odom, e_opt = rmse(odom_only), rmse(opt)
    lm_err = np.mean([np.linalg.norm(g.landmarks[k] - np.array(v))
                      for k, v in lms.items()])
    print(f"\nRMSE posición  odometría cruda : {e_odom:.4f} m")
    print(f"RMSE posición  tras GraphSLAM  : {e_opt:.4f} m")
    print(f"Error medio de landmarks       : {lm_err:.4f} m")

    ok = e_opt < e_odom and e_opt < 0.15 and lm_err < 0.20
    print("\nAUTOTEST:", "OK ✓" if ok else "FALLÓ ✗")
    return ok


if __name__ == '__main__':
    import sys
    sys.exit(0 if _synthetic_selftest() else 1)
