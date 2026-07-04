#!/usr/bin/env python3
"""Genera worlds/aruco_landmarks_casa.yaml: 48 landmarks ArUco virtuales.

Herramienta de autoria (no es un nodo ROS, se corre una vez y el YAML
resultante se versiona). Reparte los landmarks por habitacion,
proporcional al area, montados cerca de las paredes (como un ArUco real
pegado a una pared) para que el sensor virtual tenga una densidad y
distribucion comparable a los 50 marcadores reales de la Parte A
(~1.57 landmarks/m² en el laberinto real).

Geometria de paredes/habitaciones extraida a mano de
worlds/casa.world (13 paredes: 4 exteriores + 9 interiores, id de pared
Wall_NN -> segmento de linea calculado desde pose+yaw+longitud de cada
<box> de colision).
"""
import math

# Se corre desde src/turtlebot3_custom_simulation/ (donde vive la geometria de
# referencia de los .world); el YAML resultante se versiona en aruco_sim_pkg,
# que es quien lo instala/lee en runtime.
OUT_PATH = "../aruco_sim_pkg/worlds/aruco_landmarks_casa.yaml"

# Habitaciones aproximadas como rectangulos (xmin, xmax, ymin, ymax),
# derivadas de las paredes interiores de casa.world. No son polígonos
# exactos (algunas paredes no llegan a tocarse del todo) pero alcanzan
# para repartir landmarks con densidad y separacion razonables.
ZONES = [
    # name,            xmin,  xmax, ymin,  ymax, count, big_room
    ("top_right",      -0.90, 2.85, -1.45, 2.85, 17, True),
    ("top_left",       -2.85, -1.00, 0.55, 2.85, 7, False),
    ("bottom_left",    -2.85, -1.00, -1.45, 0.45, 6, False),
    ("bottom_strip_a", -2.85, -2.00, -2.85, -1.55, 4, False),
    ("bottom_strip_b", -1.90, -1.00, -2.85, -1.55, 4, False),
    ("bottom_strip_c", -0.90, 0.90, -2.85, -1.55, 5, False),
    ("bottom_strip_d", 1.00, 2.85, -2.85, -1.55, 5, False),
]

WALL_OFFSET = 0.10   # separacion del landmark respecto del borde de la habitacion
MIN_SPACING = 0.6    # separacion minima entre landmarks (id da asociacion gratis,
                      # esto es solo para que la geometria sea razonable)


def perimeter_points(xmin, xmax, ymin, ymax, n, offset=WALL_OFFSET):
    """n puntos repartidos a lo largo del perimetro del rectangulo,
    corridos `offset` hacia adentro, con theta = normal hacia el interior."""
    w = xmax - xmin
    h = ymax - ymin
    perim = 2 * (w + h)
    pts = []
    for i in range(n):
        s = (i + 0.5) * perim / n  # offset de fase para no arrancar justo en una esquina
        if s < w:
            x, y, theta = xmin + s, ymin, 90.0
        elif s < w + h:
            x, y, theta = xmax, ymin + (s - w), 180.0
        elif s < 2 * w + h:
            x, y, theta = xmax - (s - w - h), ymax, -90.0
        else:
            x, y, theta = xmin, ymax - (s - 2 * w - h), 0.0
        # empujar `offset` hacia el interior segun la normal
        if theta == 90.0:
            y += offset
        elif theta == 180.0:
            x -= offset
        elif theta == -90.0:
            y -= offset
        elif theta == 0.0:
            x += offset
        pts.append((round(x, 3), round(y, 3), theta))
    return pts


def interior_grid_points(xmin, xmax, ymin, ymax, n):
    """n puntos dispersos en el interior de una habitacion grande, para
    dar diversidad de rango (no solo landmarks pegados a la pared)."""
    pts = []
    margin = 0.8
    cols = max(1, round(math.sqrt(n)))
    rows = max(1, math.ceil(n / cols))
    xs = [xmin + margin + i * (xmax - xmin - 2 * margin) / max(1, cols - 1)
          for i in range(cols)] if cols > 1 else [(xmin + xmax) / 2]
    ys = [ymin + margin + j * (ymax - ymin - 2 * margin) / max(1, rows - 1)
          for j in range(rows)] if rows > 1 else [(ymin + ymax) / 2]
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            if len(pts) >= n:
                break
            theta = (i * 137 + j * 53) % 360  # dispersion arbitraria, no hay pared que emular
            pts.append((round(x, 3), round(y, 3), float(theta)))
    return pts[:n]


def too_close(pt, placed, min_spacing=MIN_SPACING):
    for p in placed:
        if math.hypot(pt[0] - p[0], pt[1] - p[1]) < min_spacing:
            return True
    return False


def main():
    landmarks = {}
    next_id = 0
    for name, xmin, xmax, ymin, ymax, count, big_room in ZONES:
        if big_room:
            n_perimeter = int(round(count * 0.7))
            n_interior = count - n_perimeter
            candidates = (perimeter_points(xmin, xmax, ymin, ymax, n_perimeter) +
                          interior_grid_points(xmin, xmax, ymin, ymax, n_interior))
        else:
            candidates = perimeter_points(xmin, xmax, ymin, ymax, count)

        # el espaciado minimo solo importa entre landmarks de la MISMA
        # habitacion (visibles desde el mismo punto de vista); dos
        # landmarks en habitaciones contiguas separadas por una pared
        # pueden estar geometricamente cerca sin ser un problema real.
        placed_in_zone = []
        for (x, y, theta) in candidates:
            if too_close((x, y), placed_in_zone):
                print(f"aviso: landmark en zona {name} muy cerca de otro: ({x},{y})")
            placed_in_zone.append((x, y))
            landmarks[next_id] = [x, y, theta, name]
            next_id += 1

    total = next_id
    with open(OUT_PATH, "w") as f:
        f.write("# aruco_landmarks_casa.yaml -- poses de marcadores ArUco virtuales (Sistema 3)\n")
        f.write("# id: [x, y, theta_deg]  -- x,y en frame 'map' (mismo origen que casa.world);\n")
        f.write("# theta = normal del marcador (hacia donde \"mira\"), grados, 0 = eje +x.\n")
        f.write(f"# Generado por scripts/generate_landmark_layout.py -- {total} landmarks,\n")
        f.write("# densidad ~1.4/m^2 (Parte A real: 1.57/m^2 en 31.8 m^2 con 50 marcadores).\n")
        for zone_name, *_ in [(z[0],) for z in ZONES]:
            pass
        current_zone = None
        for lid in range(total):
            x, y, theta, zone = landmarks[lid]
            if zone != current_zone:
                f.write(f"# --- {zone} ---\n")
                current_zone = zone
            f.write(f"{lid}: [{x}, {y}, {theta}]\n")

    print(f"Escritos {total} landmarks en {OUT_PATH}")


if __name__ == "__main__":
    main()
