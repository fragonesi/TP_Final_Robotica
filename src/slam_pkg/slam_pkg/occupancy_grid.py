"""Grilla de ocupación por log-odds (segunda pasada del SLAM).

Una vez que GraphSLAM corrigió la trayectoria, esta etapa proyecta las lecturas
del LIDAR sobre esa trayectoria para construir un mapa de grilla métrico — el
entregable final que usarán los planificadores (A*/Dijkstra) en las Partes B y C.

Modelo estándar de mapeo con poses conocidas (Probabilistic Robotics, cap. 9):
para cada haz del LIDAR se marcan como LIBRES las celdas que atraviesa (Bresenham)
y como OCUPADA la celda del impacto, acumulando en log-odds para que múltiples
observaciones se refuercen y el ruido disperso se promedie.

Exporta en formato map_server de ROS (.pgm + .yaml), listo para nav2.

Autotest sintético (reconstruye una sala rectangular conocida):
    python3 occupancy_grid.py
"""

import numpy as np


# Probabilidad ↔ log-odds
def _logodds(p):
    return np.log(p / (1.0 - p))


class OccupancyGridMap:
    def __init__(self, resolution=0.05, origin=(0.0, 0.0), size=(200, 200),
                 p_occ=0.7, p_free=0.4, clamp=5.0):
        """resolution [m/celda], origin = esquina inferior-izquierda [m],
        size = (ancho, alto) en celdas. p_occ/p_free: prob. inversa del sensor."""
        self.res = float(resolution)
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.w, self.h = int(size[0]), int(size[1])
        self.l = np.zeros((self.h, self.w))          # log-odds [row=y, col=x]
        self.l_occ = _logodds(p_occ)
        self.l_free = _logodds(p_free)
        self.clamp = float(clamp)

    # -- construcción automática del marco a partir de la trayectoria --------- #
    @classmethod
    def auto(cls, poses_xy, range_max, resolution=0.05, margin=1.0, **kw):
        poses_xy = np.asarray(poses_xy)
        lo = poses_xy.min(axis=0) - (range_max + margin)
        hi = poses_xy.max(axis=0) + (range_max + margin)
        size = np.ceil((hi - lo) / resolution).astype(int)
        return cls(resolution=resolution, origin=(lo[0], lo[1]),
                   size=(size[0], size[1]), **kw)

    # -- conversión mundo ↔ celda -------------------------------------------- #
    def w2c(self, x, y):
        return (int((x - self.ox) / self.res), int((y - self.oy) / self.res))  # (col, row)

    def _in_bounds(self, c, r):
        return 0 <= c < self.w and 0 <= r < self.h

    @staticmethod
    def _bresenham(c0, r0, c1, r1):
        """Celdas en la recta (c0,r0)→(c1,r1), excluyendo el extremo final."""
        cells = []
        dc, dr = abs(c1 - c0), abs(r1 - r0)
        sc, sr = (1 if c0 < c1 else -1), (1 if r0 < r1 else -1)
        err = dc - dr
        c, r = c0, r0
        while (c, r) != (c1, r1):
            cells.append((c, r))
            e2 = 2 * err
            if e2 > -dr:
                err -= dr; c += sc
            if e2 < dc:
                err += dc; r += sr
        return cells

    # -- integración de un barrido ------------------------------------------- #
    def integrate_scan(self, pose, angles, ranges, range_max, laser_offset=(0.0, 0.0, 0.0),
                       intensities=None, intensity_min=0.0, range_cap=None):
        """Acumula un barrido LIDAR. pose=(x,y,θ) del robot (base_link); angles/ranges
        arrays paralelos. laser_offset=(dx,dy,dyaw) ubica el LIDAR respecto del robot
        (del tf_static): en el TurtleBot4 el rplidar está ~4 cm atrás y ROTADO 90°.
        Un rayo a range_max (o más) se trata como 'sin retorno' → solo libera.

        Filtros para que las paredes no salgan borrosas:
          - intensities/intensity_min: si se pasan las intensidades, los haces con
            intensity <= intensity_min se descartan por completo (no liberan ni ocupan).
            En el rplidar intensity==0 marca retornos inválidos/espurios (lo mismo que
            descarta la parte0 de la cátedra).
          - range_cap: rangos por encima de este umbral se tratan como 'sin retorno'
            (liberan hasta el cap, no ocupan). Mata los chorros largos y aislados que,
            siendo poco confiables a larga distancia, ensucian el mapa. None = sin cap."""
        x, y, th = pose
        ldx, ldy, ldyaw = laser_offset
        c, s = np.cos(th), np.sin(th)
        # Origen del láser en el mundo (robot ⊕ offset) y orientación base de los rayos.
        ox = x + ldx * c - ldy * s
        oy = y + ldx * s + ldy * c
        base_ang = th + ldyaw
        c0, r0 = self.w2c(ox, oy)
        eff_max = range_max if range_cap is None else min(range_max, range_cap)
        for i, (ang, rng) in enumerate(zip(angles, ranges)):
            if intensities is not None and intensities[i] <= intensity_min:
                continue                                     # haz espurio: ni libera ni ocupa
            if not np.isfinite(rng) or rng <= 0.0:
                continue
            hit = rng < eff_max
            rr = min(rng, eff_max)
            ex = ox + rr * np.cos(base_ang + ang)
            ey = oy + rr * np.sin(base_ang + ang)
            c1, r1 = self.w2c(ex, ey)
            for (c, r) in self._bresenham(c0, r0, c1, r1):   # celdas libres
                if self._in_bounds(c, r):
                    self.l[r, c] = np.clip(self.l[r, c] + self.l_free,
                                           -self.clamp, self.clamp)
            if hit and self._in_bounds(c1, r1):              # celda del impacto
                self.l[r1, c1] = np.clip(self.l[r1, c1] + self.l_occ,
                                         -self.clamp, self.clamp)

    # -- salidas ------------------------------------------------------------- #
    def prob(self):
        """Mapa de probabilidad de ocupación [0,1]."""
        return 1.0 - 1.0 / (1.0 + np.exp(self.l))

    def to_occupancy(self, occ_thresh=0.65, free_thresh=0.25):
        """Grilla estilo nav_msgs/OccupancyGrid: 0=libre, 100=ocupado, -1=desconocido."""
        p = self.prob()
        grid = np.full(p.shape, -1, dtype=np.int8)
        grid[p >= occ_thresh] = 100
        grid[p <= free_thresh] = 0
        return grid

    def export_ros_map(self, prefix, occ_thresh=0.65, free_thresh=0.25):
        """Escribe <prefix>.pgm + <prefix>.yaml (formato map_server de ROS)."""
        p = self.prob()
        img = np.full(p.shape, 205, dtype=np.uint8)   # desconocido (gris)
        img[p <= free_thresh] = 254                    # libre (blanco)
        img[p >= occ_thresh] = 0                       # ocupado (negro)
        img = np.flipud(img)                           # PGM: fila 0 arriba
        with open(prefix + '.pgm', 'wb') as f:
            f.write(b'P5\n%d %d\n255\n' % (self.w, self.h))
            f.write(img.tobytes())
        with open(prefix + '.yaml', 'w') as f:
            f.write(f"image: {prefix.split('/')[-1]}.pgm\n")
            f.write(f"resolution: {self.res}\n")
            f.write(f"origin: [{self.ox}, {self.oy}, 0.0]\n")
            f.write("negate: 0\n")
            f.write(f"occupied_thresh: {occ_thresh}\n")
            f.write(f"free_thresh: {free_thresh}\n")
        return prefix + '.pgm', prefix + '.yaml'


# --------------------------------------------------------------------------- #
# Integración con datos reales: interpolar pose por timestamp y proyectar scans
# --------------------------------------------------------------------------- #
def pose_at_time(poses, times, t):
    """Pose corregida interpolada en el instante t (x,y lineal; θ por sen/cos)."""
    poses, times = np.asarray(poses), np.asarray(times)
    if t <= times[0]:
        return poses[0]
    if t >= times[-1]:
        return poses[-1]
    i = int(np.searchsorted(times, t))
    t0, t1 = times[i - 1], times[i]
    a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    p0, p1 = poses[i - 1], poses[i]
    s = (1 - a) * np.sin(p0[2]) + a * np.sin(p1[2])
    c = (1 - a) * np.cos(p0[2]) + a * np.cos(p1[2])
    return np.array([p0[0] + a * (p1[0] - p0[0]),
                     p0[1] + a * (p1[1] - p0[1]),
                     np.arctan2(s, c)])


# --------------------------------------------------------------------------- #
# Scan-matching contra el mapa (estilo Hector SLAM): refina la pose de cada
# barrido para que sus impactos "calcen" sobre las paredes ya consensuadas. Así
# se cancela el jitter de pose (interpolación + drift residual) que engorda las
# paredes — el grafo da la estructura global (con loop closure), esto afina lo local.
# --------------------------------------------------------------------------- #
class LikelihoodField:
    """Campo de verosimilitud 2D (occupancy suavizada) con muestreo bilineal y
    gradiente analítico, para alinear barridos por Gauss-Newton."""
    def __init__(self, field, res, ox, oy):
        self.M = field                      # [row=y, col=x] en [0,1]
        self.res, self.ox, self.oy = res, ox, oy
        self.h, self.w = field.shape

    @classmethod
    def from_endpoints(cls, exs, eys, res, ox, oy, shape, blur):
        """Acumula los impactos (endpoints) en una grilla, los desenfoca (Gaussiana)
        y normaliza → cuenca suave alrededor de cada pared para que el matcher
        converja. blur en celdas (más grande = más radio de captura, menos preciso)."""
        from scipy.ndimage import gaussian_filter
        h, w = shape
        col = np.round((exs - ox) / res).astype(int)
        row = np.round((eys - oy) / res).astype(int)
        m = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        f = np.zeros((h, w))
        np.add.at(f, (row[m], col[m]), 1.0)
        f = np.minimum(f, 1.0)               # presencia de pared, no densidad
        f = gaussian_filter(f, blur)
        if f.max() > 0:
            f /= f.max()
        return cls(f, res, ox, oy)

    def sample(self, xw, yw):
        """M y su gradiente (∂M/∂x, ∂M/∂y en 1/m) en puntos mundo, bilineal y vectorizado."""
        cx = np.clip((xw - self.ox) / self.res, 0, self.w - 1.001)
        cy = np.clip((yw - self.oy) / self.res, 0, self.h - 1.001)
        x0 = np.floor(cx).astype(int); y0 = np.floor(cy).astype(int)
        fx = cx - x0; fy = cy - y0
        M = self.M
        v00 = M[y0, x0]; v01 = M[y0, x0 + 1]; v10 = M[y0 + 1, x0]; v11 = M[y0 + 1, x0 + 1]
        val = (v00 * (1 - fx) * (1 - fy) + v01 * fx * (1 - fy)
               + v10 * (1 - fx) * fy + v11 * fx * fy)
        dcol = (v01 - v00) * (1 - fy) + (v11 - v10) * fy       # ∂M/∂col
        drow = (v10 - v00) * (1 - fx) + (v11 - v01) * fx       # ∂M/∂row
        return val, dcol / self.res, drow / self.res


def scan_match_pose(field, pose, r, beta, laser_offset, iters=6,
                    max_dtrans=0.30, max_dtheta=np.deg2rad(12.0)):
    """Refina pose=(x,y,θ) maximizando Σ M(endpoint_i) por Gauss-Newton. r/beta son
    rango y ángulo (β=ldyaw+ang, así φ_mundo=θ+β) de los haces VÁLIDOS. Devuelve la
    pose corregida, o la original si la corrección es exagerada (match dudoso) o hay
    pocos haces. Pasos acotados por iteración para no divergir."""
    ldx, ldy, _ = laser_offset
    x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
    x0, y0, th0 = x, y, th
    if len(r) < 20:
        return (x0, y0, th0)
    for _ in range(iters):
        c, s = np.cos(th), np.sin(th)
        phi = th + beta
        cb, sb = np.cos(phi), np.sin(phi)
        ex = x + ldx * c - ldy * s + r * cb
        ey = y + ldx * s + ldy * c + r * sb
        M, Mx, My = field.sample(ex, ey)
        dex_dth = -ldx * s - ldy * c - r * sb
        dey_dth = ldx * c - ldy * s + r * cb
        # J_i = ∂M/∂ξ = [Mx, My, Mx·∂ex/∂θ + My·∂ey/∂θ]; residuo = 1 - M
        J = np.stack([Mx, My, Mx * dex_dth + My * dey_dth], axis=1)
        res = 1.0 - M
        H = J.T @ J
        H += 1e-2 * np.diag(np.diag(H)) + 1e-6 * np.eye(3)     # damping tipo LM (escala-invariante)
        g = J.T @ res
        try:
            d = np.linalg.solve(H, g)
        except np.linalg.LinAlgError:
            break
        d[:2] = np.clip(d[:2], -0.05, 0.05)                    # cap por iteración
        d[2] = np.clip(d[2], -np.deg2rad(3), np.deg2rad(3))
        x += d[0]; y += d[1]; th += d[2]
        if abs(d[0]) < 1e-4 and abs(d[1]) < 1e-4 and abs(d[2]) < 1e-4:
            break
    dth = np.arctan2(np.sin(th - th0), np.cos(th - th0))
    if np.hypot(x - x0, y - y0) > max_dtrans or abs(dth) > max_dtheta:
        return (x0, y0, th0)                                   # rechazo: match poco fiable
    return (x, y, th)


def build_grid_from_scans(poses, pose_times, scans_csv, resolution=0.05, max_scans=4000,
                          laser_offset=(-0.04, 0.0, np.pi / 2),
                          range_cap=5.0, intensity_min=0.0,
                          scan_match=False, sm_passes=2, sm_iters=6, sm_blur0=3.0):
    """Proyecta los barridos del LIDAR sobre la trayectoria corregida y devuelve la
    grilla. scans_csv: salida de scan_logger_node. Submuestrea a <= max_scans para
    acotar el costo (más barridos → paredes mejor consensuadas, menos borrón).
    laser_offset=(dx,dy,dyaw) del LIDAR respecto de base_link; el default es la
    extrínseca del rplidar del TurtleBot4 (del tf_static del bag: ~4 cm atrás y rotado
    90° en yaw) — clave para que las paredes no salgan borrosas.

    Limpieza de haces (ver integrate_scan):
      - range_cap [m]: descarta como impacto los rangos largos (poco confiables, son
        los chorros aislados que ensucian); el laberinto entra de sobra en ~5 m.
      - intensity_min: si el CSV trae columnas de intensidad (i0..iN, las guarda
        scan_logger_node), descarta los haces con intensity <= intensity_min (los
        retornos inválidos del rplidar son intensity==0). Si no hay columnas i*, se
        ignora y no se filtra por intensidad.

    scan_match: si True, refina la pose de cada barrido por scan-matching contra el
    mapa (Gauss-Newton sobre un likelihood field), en sm_passes pasadas coarse-to-fine
    (blur decreciente). Corrige el jitter de pose → paredes más finas. Requiere pandas/scipy."""
    import pandas as pd
    df = pd.read_csv(scans_csv)
    step = max(1, len(df) // max_scans)
    df = df.iloc[::step]
    poses, pose_times = np.asarray(poses), np.asarray(pose_times)
    rcols = [c for c in df.columns if c.startswith('r') and c[1:].isdigit()]
    icols = [c for c in df.columns if c.startswith('i') and c[1:].isdigit()]
    has_int = len(icols) == len(rcols)
    range_max = float(df['range_max'].iloc[0])
    eff_max = range_max if range_cap is None else min(range_max, range_cap)

    grid = OccupancyGridMap.auto(poses[:, :2], range_max=range_max, resolution=resolution)
    ldx, ldy, ldyaw = laser_offset
    n = len(rcols)

    # Pre-parseo: por barrido guardamos (pose_slam, angles, ranges, intens, máscara de
    # haces válidos para matching). Evita re-leer el DataFrame en cada pasada.
    recs = []
    for _, row in df.iterrows():
        angles = row['angle_min'] + np.arange(n) * row['angle_increment']
        ranges = row[rcols].to_numpy(dtype=float)
        intens = row[icols].to_numpy(dtype=float) if has_int else None
        pose = pose_at_time(poses, pose_times, row['timestamp'])
        valid = np.isfinite(ranges) & (ranges > 0.0) & (ranges < eff_max)
        if intens is not None:
            valid &= intens > intensity_min
        recs.append((pose, angles, ranges, intens, valid))

    sm_poses = [rec[0] for rec in recs]
    if scan_match:
        for p in range(sm_passes):
            blur = max(1.0, sm_blur0 / (p + 1))     # coarse → fine
            # 1) likelihood field desde los impactos en las poses actuales
            exs, eys = [], []
            for (pose, angles, ranges, intens, valid), pse in zip(recs, sm_poses):
                x, y, th = pse
                phi = th + ldyaw + angles[valid]
                rr = ranges[valid]
                ox = x + ldx * np.cos(th) - ldy * np.sin(th)
                oy = y + ldx * np.sin(th) + ldy * np.cos(th)
                exs.append(ox + rr * np.cos(phi)); eys.append(oy + rr * np.sin(phi))
            field = LikelihoodField.from_endpoints(
                np.concatenate(exs), np.concatenate(eys),
                grid.res, grid.ox, grid.oy, grid.l.shape, blur)
            # 2) refinar cada pose contra ese campo
            new_poses = []
            for (pose, angles, ranges, intens, valid), pse in zip(recs, sm_poses):
                beta = ldyaw + angles[valid]
                new_poses.append(scan_match_pose(field, pse, ranges[valid], beta,
                                                 laser_offset, iters=sm_iters))
            sm_poses = new_poses

    # Integración final (Bresenham, con todos los filtros) en las poses refinadas.
    for (pose, angles, ranges, intens, valid), pse in zip(recs, sm_poses):
        grid.integrate_scan(pse, angles, ranges, range_max,
                            laser_offset=laser_offset, intensities=intens,
                            intensity_min=intensity_min, range_cap=range_cap)
    return grid


# --------------------------------------------------------------------------- #
# Autotest sintético: reconstruir una sala rectangular con un pasaje interno
# --------------------------------------------------------------------------- #
def _raycast(pose, angles, segments, range_max):
    """Simula un barrido LIDAR contra segmentos de pared (verdad-terreno)."""
    x, y, th = pose
    ranges = np.full(len(angles), range_max)
    for i, ang in enumerate(angles):
        dx, dy = np.cos(th + ang), np.sin(th + ang)
        best = range_max
        for (ax, ay, bx, by) in segments:
            # intersección rayo (x,y)+t*(dx,dy) con segmento a→b
            ex, ey = bx - ax, by - ay
            denom = dx * (-ey) - dy * (-ex)
            if abs(denom) < 1e-12:
                continue
            t = ((ax - x) * (-ey) - (ay - y) * (-ex)) / denom
            u = (dx * (ay - y) - dy * (ax - x)) / denom
            if t > 0 and 0.0 <= u <= 1.0 and t < best:
                best = t
        ranges[i] = best
    return ranges


def _synthetic_selftest():
    rng = np.random.default_rng(0)
    # Sala 10x8 con una pared interna (verdad-terreno como segmentos).
    segs = [
        (0, 0, 10, 0), (10, 0, 10, 8), (10, 8, 0, 8), (0, 8, 0, 0),  # perímetro
        (5, 0, 5, 5),                                                 # tabique interno
    ]
    range_max = 12.0
    angles = np.linspace(-np.pi, np.pi, 180, endpoint=False)

    # Trayectoria del robot recorriendo la sala.
    path = [(2, 2, 0), (4, 2, 0.3), (3, 5, 1.0), (7, 6, 0.0), (8, 3, -0.5), (6, 2, 3.0)]

    grid = OccupancyGridMap.auto([(p[0], p[1]) for p in path],
                                 range_max=range_max, resolution=0.10)
    for pose in path:
        ranges = _raycast(pose, angles, segs, range_max)
        ranges = ranges + rng.normal(0, 0.02, size=ranges.shape)   # ruido del sensor
        grid.integrate_scan(pose, angles, ranges, range_max)

    p = grid.prob()

    # Verificación: muestrear puntos sobre las paredes → deberían dar ocupados;
    # puntos del interior libre → deberían dar libres.
    def cell_prob(x, y, rad=0):
        # Con paredes de 1 celda, el impacto puede caer en la celda contigua
        # (según el lado del haz); para paredes miramos el máx en ±rad celdas.
        c, r = grid.w2c(x, y)
        vals = [p[r + dr, c + dc]
                for dc in range(-rad, rad + 1) for dr in range(-rad, rad + 1)
                if grid._in_bounds(c + dc, r + dr)]
        return max(vals) if vals else 0.5

    wall_pts, free_pts = [], []
    for (ax, ay, bx, by) in segs:
        for s in np.linspace(0.05, 0.95, 25):
            wall_pts.append(cell_prob(ax + s * (bx - ax), ay + s * (by - ay), rad=1))
    for (fx, fy) in [(2, 4), (3, 3), (7, 4), (8, 5), (6, 5), (2, 6)]:
        free_pts.append(cell_prob(fx, fy))

    wall_occ = np.mean([q > 0.6 for q in wall_pts])
    free_ok = np.mean([q < 0.4 for q in free_pts])
    print(f"paredes recuperadas como ocupadas: {wall_occ*100:.0f}%")
    print(f"interior recuperado como libre   : {free_ok*100:.0f}%")

    pgm, yaml = grid.export_ros_map('/tmp/occ_selftest')
    print(f"mapa exportado: {pgm}, {yaml}")

    ok = wall_occ > 0.7 and free_ok > 0.8
    print("\nAUTOTEST:", "OK ✓" if ok else "FALLÓ ✗")
    return ok


if __name__ == '__main__':
    import sys
    sys.exit(0 if _synthetic_selftest() else 1)
