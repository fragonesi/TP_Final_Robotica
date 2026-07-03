# Estado del proyecto — Parte A (Opción 3)

> Actualización de avances y handoff. Última actualización: 2026-06-30.
> Este repo contiene el workspace completo de la Parte A bajo `src/`:
> `src/aruco_pkg` (percepción ArUco, odometría y modelo de ruido) y
> `src/TP_Final_Robotica` (**GraphSLAM**).

## Avances realizados

- **Rosbag**: configurado y reproduciéndose correctamente (`ros2 bag play`).
- **Percepción + odometría** (en `src/aruco_pkg`): detección de marcadores ArUco
  (`aruco_detector_node`) y registro de odometría con el modelo de deltas
  `(δrot1, δtrans, δrot2)` (`odom_delta_node`) ya funcionando.
- **Exportación a CSV y gráficos**: el bag del laberinto dura ~24 min, así que se
  dejó procesando para no correrlo de cero cada vez. Los CSV ya generados quedan
  versionados en `src/aruco_pkg` (`aruco_detections.csv`, `odom_deltas.csv`).
- **Bugs resueltos**: funcionamiento interno y, sobre todo, el diccionario ArUco.
  El correcto es **`DICT_4X4_50`** (no `DICT_5X5_250`, que detectaba candidatos
  pero no decodificaba ningún ID). Detalle completo en `TpParteA.md` (raíz del repo).

## Hecho (sesión de implementación de GraphSLAM)

Todo está en `src/TP_Final_Robotica/TP_Final_Robotica/` y commiteado en la branch `parte-A`.

- [x] **Núcleo de GraphSLAM** — `graph_slam.py`: optimizador batch por mínimos
  cuadrados (Gauss-Newton disperso), edges de odometría (sobre los deltas) +
  edges de observación rango-bearing con covarianza dependiente de la distancia.
  Disperso (scipy) + submuestreo a keyframes → escala a las 28k lecturas del
  laberinto. Sin g2o/GTSAM (autocontenido).
  *Autotest:* odometría cruda ~0.45 m RMSE → ~0.04 m tras optimizar (cierre de lazo OK).
  El borrador previo `gslam.py` (forma de información estilo Thrun) queda como referencia.
- [x] **Pipeline "corré-todo"** — `slam_pipeline.py`: de los CSV a los entregables
  (`landmarks.json` por ID + `trayectoria_opt.csv`) + gráfico de diagnóstico.
  Verificado sobre la odometría real del laberinto (772 keyframes).
- [x] **Nodo ROS para RViz** — `graph_slam_node.py`: optimiza y publica `/belief`
  (Path), `/landmarks` (MarkerArray por ID) y `/poses_guardadas` (PoseArray).
  Registrado como entry point en `setup.py`. Import verificado bajo Humble.
- [x] **Grilla de ocupación (2da pasada)** — `occupancy_grid.py`: mapeo log-odds +
  Bresenham, exporta a formato map_server de ROS (`.pgm`/`.yaml`).
  *Autotest:* reconstruye una sala conocida (100% paredes, 100% interior libre).
- [x] **Fix de paquete**: `package.xml` declaraba `tpf` (inconsistente con el módulo,
  `setup.py` y el marcador de recurso, rompía colcon) → alineado a `TP_Final_Robotica`,
  + dependencias agregadas (rclpy, geometry/nav/visualization_msgs, numpy/scipy/pandas).

## Hecho (corrida sobre el laberinto real + afinado del mapa)

- [x] **Corrido end-to-end sobre el bag del laberinto** (23 min): detector ArUco →
  CSV → GraphSLAM → entregables. Cierre de lazo real (las detecciones ya son del
  laberinto, no del bag corto). Resultado: **50 landmarks** coherentes, χ² baja con
  gating, trayectoria corregida sensata. Salida en `Rosbags/full_run/slam_out/`.
- [x] **Grilla de ocupación con datos reales**: `scan_logger_node` loguea `tb4_0/scan`
  (QoS BEST_EFFORT) y `occupancy_grid` proyecta los barridos sobre la trayectoria
  corregida (pose interpolada por timestamp) → mapa `.pgm`/`.yaml` exportado.
- [x] **Extrínseca cámara→robot** aplicada (del `tf_static`): rotación óptico→base +
  traslación (~6 cm atrás). La grande era la del LIDAR (+90°), también aplicada.
- [x] **Afinado del mapa** (29/06): se sacaron los chorros espurios largos con un
  **cap de rango** (~5 m; el laberinto entra de sobra, el 99% de los returns son <3.2 m),
  se subió la **cobertura de barridos** (800 → 4000) para consensuar mejor las paredes,
  y se agregó **filtrado por intensidad** (`scan_logger` ahora guarda `i0..iN`;
  `occupancy_grid` descarta `intensity<=umbral`). Nota: en este rplidar `intensity==0`
  coincide exactamente con los haces sin retorno (inf), que ya se descartaban — el
  filtro queda como red de seguridad, alineado con la parte0 de la cátedra.

- [x] **Scan-matching contra el mapa** (29/06): para el borrón **residual** de las
  paredes (que es **error de pose** durante los scans, no ruido) se agregó un refinador
  por barrido estilo Hector SLAM (`LikelihoodField` + Gauss-Newton, pasadas coarse-to-fine,
  con rechazo de matches dudosos) en `occupancy_grid.py`. Adelgaza el chorro central y
  las paredes (ocupadas 3180 → ~2700). Flag `--sm-passes` (default 2; el entregable usa 3).
  Honesto: el grueso de la mejora viene con 2 pasadas; sigue sin ser un laberinto perfecto.

## Hecho (30/06 — NoiseModel enchufado)

- [x] **`NoiseModel` ajustado enchufado en el GraphSLAM**: el `noise_model.json`
  fiteado por `fit_noise_model.py` ahora se propaga como covarianza real de cada
  edge ArUco (cartesiano (tx,tz) → rango-bearing vía Jacobiano), con término
  sistemático de escala (`scale_uncertainty·r`) para no sobre-confiar el rango.
  - `aruco_pkg/setup.py`: `noise_model.json` instalado a `share/aruco_pkg/` via colcon.
  - `graph_slam_node.py`: nuevo param `noise_model_path`; se auto-detecta desde
    `share/aruco_pkg/` si no se pasa explícitamente.
  - `slam.launch.py`: nuevo arg `noise_model_path` con default auto-detectado.
  - `slam_pipeline.py`: ya tenía `--noise-model`; README actualizado para usarlo.

## Pendiente
- [ ] **Nitidez final (opcional, agregado grande)**: para el salto final de paredes
  finas haría falta meter edges de **scan-matching dentro del GraphSLAM** (no solo en la
  2da pasada de la grilla).
- [ ] **Cierre de entrega**: empaquetar el mapa final + `landmarks.json` y la integración
  con la Parte B. (Launch files y config `.rviz` ya están en el repo.)

## ⚠️ Importante: el bag del laberinto NO está incluido

El rosbag del laberinto pesa ~8 GB, así que **se quitó del zip** que circuló entre
nosotras y **no va a git** (`.gitignore` excluye `*.db3`/`*.mcap`). Hay que tenerlo
en local para regenerar el mapa. (El bag corto `aruco_estimation`, ~184 MB, tampoco
se versiona; solo sirve para caracterizar el modelo de ruido, no para mapear.)
Los CSV ya generados del laberinto (`odom_deltas.csv`, `laberinto_detections.csv`,
`scans.csv`) quedan en `Rosbags/full_run/`, así que el pipeline se puede re-correr
sin volver a reproducir el bag.

## Documentación

- [x] `README.md`: build + corrida end-to-end con los comandos para copiar/pegar.
- [x] `TpParteA.md`: bitácora detallada (decisiones, bugs y cómo se resolvieron).
