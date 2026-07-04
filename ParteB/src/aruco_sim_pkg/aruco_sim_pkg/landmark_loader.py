"""Carga el layout de landmarks ArUco virtuales desde YAML.

Formato (ver worlds/aruco_landmarks_casa.yaml, generado por
turtlebot3_custom_simulation/scripts/generate_landmark_layout.py):
    <id:int>: [x, y, theta_deg]
x,y en el frame 'map' (mismo origen que el .world); theta = normal del
marcador (hacia donde "mira"), en grados, 0 = eje +x.
"""
import math
from collections import namedtuple

import yaml

Landmark = namedtuple('Landmark', ['id', 'x', 'y', 'theta'])


def load_landmarks(yaml_path):
    """Devuelve {id: Landmark} con theta en radianes."""
    with open(yaml_path, 'r') as f:
        raw = yaml.safe_load(f)
    landmarks = {}
    for lid, (x, y, theta_deg) in raw.items():
        lid = int(lid)
        landmarks[lid] = Landmark(lid, float(x), float(y), math.radians(float(theta_deg)))
    return landmarks
