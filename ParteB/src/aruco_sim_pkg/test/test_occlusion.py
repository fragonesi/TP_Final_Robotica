"""Tests puros de geometria para occlusion.py (sin ROS)."""
from aruco_sim_pkg.occlusion import (
    get_obstacles, is_occluded, point_segment_distance, segments_intersect,
)


def test_segments_intersect_crossing():
    assert segments_intersect((0, 0), (2, 2), (0, 2), (2, 0))


def test_segments_intersect_parallel_no_cross():
    assert not segments_intersect((0, 0), (1, 0), (0, 1), (1, 1))


def test_point_segment_distance_perpendicular():
    # distancia de (0,1) al segmento horizontal (0,0)-(2,0) es 1
    assert abs(point_segment_distance((0, 1), (0, 0), (2, 0)) - 1.0) < 1e-9


def test_point_segment_distance_beyond_endpoint():
    # el punto mas cercano es el extremo, no una extrapolacion de la recta
    d = point_segment_distance((5, 0), (0, 0), (1, 0))
    assert abs(d - 4.0) < 1e-9


def test_landmark_behind_interior_wall_is_occluded():
    walls, furniture = get_obstacles('casa')
    # robot en la habitacion grande (top_right), landmark en top_left,
    # separados por Wall_40 (x=-0.95, y en [-1.55, 2.0]).
    robot_xy = (0.0, 1.0)
    landmark_xy = (-2.0, 1.0)
    assert is_occluded(robot_xy, landmark_xy, walls, furniture)


def test_landmark_same_room_no_obstacle_is_visible():
    walls, furniture = get_obstacles('casa')
    # ambos puntos dentro de la habitacion grande (top_right), sin pared ni
    # mueble entre medio (evita a proposito la Mesa/Sofa/HospitalBot, que
    # tambien estan en esa habitacion).
    robot_xy = (0.0, -1.3)
    landmark_xy = (2.5, -1.3)
    assert not is_occluded(robot_xy, landmark_xy, walls, furniture)


def test_furniture_occludes_only_in_casa_o():
    # Suitcase1 (-0.358, 0.793, r=0.3) solo existe en 'casa_o'.
    robot_xy = (-0.358, 1.6)
    landmark_xy = (-0.358, 0.0)  # el segmento pasa justo por el centro del mueble

    walls_casa, furniture_casa = get_obstacles('casa')
    assert not is_occluded(robot_xy, landmark_xy, walls_casa, furniture_casa)

    walls_o, furniture_o = get_obstacles('casa_o')
    assert is_occluded(robot_xy, landmark_xy, walls_o, furniture_o)


def test_unknown_world_name_raises():
    try:
        get_obstacles('no_existe')
        assert False, "deberia haber lanzado ValueError"
    except ValueError:
        pass
