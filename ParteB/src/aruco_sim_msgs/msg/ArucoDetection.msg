# Una deteccion (real o virtual) de un marcador ArUco.
#
# tx, ty, tz: en la misma convencion de frame optico de camara que usa
# aruco_pkg (aruco_detector_node.py) y que consume graph_slam.py
# (x-derecha, y-abajo, z-adelante), para que ambos "sensores" -- el real
# de Parte A y el virtual de Sistema 3 -- sean intercambiables aguas
# abajo. range/bearing se incluyen ya calculados en frame base_link
# (x-adelante, y-izquierda) porque el sensor virtual los conoce
# directamente (no hace falta resolver PnP).

uint32 marker_id
float64 tx
float64 ty
float64 tz
float64 range
float64 bearing
