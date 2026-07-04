"""Oclusion por linea de vision entre el robot y un landmark ArUco virtual.

En vez de un raycast contra la grilla de ocupacion (esa grilla hoy solo existe
para `casa.world` sin muebles, y generarla justamente es el resultado final de
Sistema 3, no un prerequisito) o un plugin de raytracing de Gazebo (no
expuesto por `gazebo_ros_pkgs` en Humble, requeriria un plugin C++ propio),
la oclusion se resuelve con geometria analitica: las paredes de
`casa.world`/`casa_o.world` son cajas SDF con pose/tamano conocidos -> se
precalculan una vez como segmentos de linea (la centerline de cada pared);
los muebles (mallas, no cajas) se aproximan como circulos de radio fijo por
tipo de modelo. Un landmark esta ocluido si el segmento robot-landmark corta
alguna pared o pasa a distancia < radio de algun mueble.

Las poses de pared/mueble estan hardcodeadas (extraidas a mano de los
.world, ver comentarios) en vez de parsear el SDF en runtime: mismo
resultado, sin acoplarse a detalles de formato SDF que podrian cambiar entre
versiones de Gazebo.
"""
import math

# --------------------------------------------------------------------------- #
# Paredes de casa.world / casa_o.world (mismo plano de planta en ambos).
# Cada segmento = centerline de una caja de colision <box><size>L 0.1 1</size>,
# calculada desde su pose (x, y, yaw) y longitud L: extremos en
# (x,y) + R(yaw)*(±L/2, 0).
# --------------------------------------------------------------------------- #
WALL_SEGMENTS = [
    ((-2.95, 3.0), (-2.95, -3.0)),   # Wall_20 (exterior, izquierda)
    ((-3.0, -2.95), (3.0, -2.95)),   # Wall_21 (exterior, abajo)
    ((2.95, -3.0), (2.95, 3.0)),     # Wall_22 (exterior, derecha)
    ((2.5, 2.95), (-2.5, 2.95)),     # Wall_23 (exterior, arriba)
    ((-3.0, -1.5), (2.0, -1.5)),     # Wall_25 (interior, divide franja inferior)
    ((-0.95, -3.0), (-0.95, -2.5)),  # Wall_29
    ((0.0, -1.45), (0.0, -2.0)),     # Wall_31
    ((0.95, -3.0), (0.95, -2.5)),    # Wall_33
    ((1.95, -1.45), (1.95, -2.0)),   # Wall_35
    ((-1.95, -1.45), (-1.95, -2.0)),  # Wall_38
    ((-0.95, -1.55), (-0.95, 2.0)),  # Wall_40 (interior, divide columna izquierda)
    ((-3.0, 0.5), (-2.25, 0.5)),     # Wall_42
    ((-0.9, 0.5), (-1.65, 0.5)),     # Wall_44
]

# Muebles presentes en AMBOS mundos (casa.world ya viene amoblado con mesa,
# sofa y el HospitalBot; casa_o.world hereda el mismo layout base).
# (cx, cy, radio) -- radio = circulo acotante aproximado del footprint del
# modelo (mesa 1.5x0.8m, sofa ~1.6x0.7m, HospitalBot caja de colision 0.7x0.7m).
FURNITURE_BASE = [
    (1.442, -0.044, 0.85),   # Table
    (0.999, 1.326, 0.65),    # Sofa
    (-1.924, -0.489, 0.5),   # HospitalBot
]

# Obstaculos adicionales SOLO en casa_o.world (valijas sueltas = los
# "obstaculos" que da nombre al mundo _o, ademas del amueblado base).
FURNITURE_CASA_O_EXTRA = [
    (-0.358, 0.793, 0.3),    # Suitcase1
    (2.172, -0.257, 0.3),    # Suitcase1_0
    (-0.037, -2.184, 0.3),   # Suitcase2H
    (-2.141, 1.930, 0.3),    # Suitcase2H_0
    (-1.888, 1.925, 0.3),    # Suitcase2H_0_clone
    (-1.618, 1.929, 0.3),    # Suitcase2H_0_clone_0
]


def get_obstacles(world_name):
    """Devuelve (walls, furniture_circles) para 'casa' o 'casa_o'.

    Los muebles se re-evaluan cada llamada a is_occluded() contra la pose
    *actual* pasada por el caller (no un snapshot cacheado a este nivel),
    asi que si algun dia se agrega un obstaculo dinamico (hoy no hay
    ninguno: todo el mobiliario es estatico) alcanza con pasar sus poses
    actualizadas -- no hace falta tocar la logica de oclusion.
    """
    furniture = list(FURNITURE_BASE)
    if world_name == 'casa_o':
        furniture += FURNITURE_CASA_O_EXTRA
    elif world_name != 'casa':
        raise ValueError(f"world_name desconocido: {world_name!r} (esperado 'casa' o 'casa_o')")
    return WALL_SEGMENTS, furniture


def _orientation(a, b, c):
    val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
    if val > 1e-12:
        return 1
    if val < -1e-12:
        return -1
    return 0


def _on_segment(a, b, p):
    return (min(a[0], b[0]) - 1e-9 <= p[0] <= max(a[0], b[0]) + 1e-9 and
            min(a[1], b[1]) - 1e-9 <= p[1] <= max(a[1], b[1]) + 1e-9)


def segments_intersect(p1, p2, p3, p4):
    """Determina si el segmento p1-p2 corta al segmento p3-p4.

    Test por orientacion, con manejo de los casos colineales.
    """
    o1 = _orientation(p1, p2, p3)
    o2 = _orientation(p1, p2, p4)
    o3 = _orientation(p3, p4, p1)
    o4 = _orientation(p3, p4, p2)

    if o1 != o2 and o3 != o4:
        return True
    if o1 == 0 and _on_segment(p1, p2, p3):
        return True
    if o2 == 0 and _on_segment(p1, p2, p4):
        return True
    if o3 == 0 and _on_segment(p3, p4, p1):
        return True
    if o4 == 0 and _on_segment(p3, p4, p2):
        return True
    return False


def point_segment_distance(point, seg_a, seg_b):
    """Distancia minima de `point` al segmento seg_a-seg_b."""
    px, py = point
    ax, ay = seg_a
    bx, by = seg_b
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    proj_x, proj_y = ax + t * dx, ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def is_occluded(robot_xy, landmark_xy, walls, furniture_circles):
    """Determina si algo se interpone entre robot_xy y landmark_xy."""
    for (wa, wb) in walls:
        if segments_intersect(robot_xy, landmark_xy, wa, wb):
            return True
    for (cx, cy, radius) in furniture_circles:
        if point_segment_distance((cx, cy), robot_xy, landmark_xy) <= radius:
            return True
    return False
