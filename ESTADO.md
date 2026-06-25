# Estado del proyecto — Parte A (Opción 3)

> Actualización de avances y handoff. Última actualización: 2026-06-25.
> Este repo (`TP_Final_Robotica`) contiene la parte de **GraphSLAM**. La detección
> ArUco, la odometría y el modelo de ruido viven en el paquete hermano `aruco_pkg`
> (Parte A de percepción), que se comparte aparte.

## Avances realizados

- **Rosbag**: configurado y reproduciéndose correctamente (`ros2 bag play`).
- **Percepción + odometría** (en `aruco_pkg`): detección de marcadores ArUco
  (`aruco_detector_node`) y registro de odometría con el modelo de deltas
  `(δrot1, δtrans, δrot2)` (`odom_delta_node`) ya funcionando.
- **Exportación a CSV y gráficos**: el bag del laberinto dura ~24 min, así que se
  dejó procesando para no correrlo de cero cada vez. Los CSV ya generados quedan
  versionados en `aruco_pkg` (`aruco_detections.csv`, `odom_deltas.csv`).
- **Bugs resueltos**: funcionamiento interno y, sobre todo, el diccionario ArUco.
  El correcto es **`DICT_4X4_50`** (no `DICT_5X5_250`, que detectaba candidatos
  pero no decodificaba ningún ID). Detalle completo en `TpParteA.md` de `aruco_pkg`.

## Siguientes pasos

- **Implementar GraphSLAM** apoyándose en la base de odometría que ya funciona.
  El borrador está en [`gslam.py`](TP_Final_Robotica/gslam.py); hoy **no es
  ejecutable ni está integrado** (faltan imports, usa un mensaje `ArucoDetection`
  que aún no existe, y espera observaciones range-bearing mientras el detector
  entrega pose 3D cartesiana). Recordar que la consigna exige **loop closure**.
- Publicar en los tópicos canónicos que esperan los docentes en RViz:
  `/belief`, `/landmarks`, `/poses_guardadas`, `/map`.

## ⚠️ Importante: el bag del laberinto NO está incluido

El rosbag del laberinto pesa ~8 GB, así que **se quitó del zip** que circuló entre
nosotras. Hay que **descargarlo aparte y colocarlo en el paquete** antes de poder
generar el mapa final. (El bag corto `aruco_estimation`, ~184 MB, tampoco se
versiona por tamaño; solo sirve para caracterizar el modelo de ruido, no para mapear.)

## Documentación pendiente

- `README.md` con el paso a paso y los comandos exactos para copiar y pegar en la
  terminal. Lo provee la compañera (en camino).
