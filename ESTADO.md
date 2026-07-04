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

## Hecho (02/07 — corrida nueva del laberinto + rescate de obstáculos finos)

- [x] **Corrida nueva procesada de punta a punta**: `rosbag2_2026_07_01-12_45_54`
  (46,8 min, bajada de Drive, extraída en `Robotica/Rosbags/`). Percepción sobre el
  bag crudo (reproducido a 2x, sin pérdidas): 55.759 deltas de odom, 19.926 scans,
  2.043 detecciones con 48 IDs. GraphSLAM: 2.195 keyframes, **46 landmarks**,
  χ² 54.442 → 1.686, gating descartó solo 2 obs. Salidas en
  `Robotica/Rosbags/corrida2_run/` (`slam_out_v2/` = mapa bueno).
  Ojo: `laberinto_filtrado1/2` (y el zip de Descargas) son **tres copias del mismo
  filtrado corto** de la corrida de las 12:30 — no hay una segunda bag filtrada.
- [x] **Rescate de obstáculos finos (`--min-hits-occ`)**: la silla junto al pouf no
  aparecía en el mapa — sus patas (~2 cm) no llenan la celda de 5 cm y el consenso
  log-odds las borraba (≈7 rayos que atraviesan por cada impacto; rebalancear
  `p_occ`/`p_free` y hasta resolución 2,5 cm no alcanzó). Ahora la grilla cuenta
  impactos crudos por celda y el export fuerza como ocupadas las celdas con ≥N
  impactos aisladas de paredes consensuadas (no engrosa el halo). Con los 19.926
  scans y N=500 rescata 30 celdas: las 4 patas + tramos de paredes finas.
  Detalles y mediciones en `TpParteA.md` (02/07). También quedaron expuestos
  `--p-occ`/`--p-free` y el `mapa.png` ahora muestra la ocupación exportada.

## Hecho (03/07 — fix del clamp de log-odds: paredes internas del laberinto recuperadas)

- [x] **Bug encontrado y arreglado**: `entrega_parte_A/mapa.pgm` capturaba bien el
  perímetro del laberinto pero casi ninguna pared interna, pese a que
  `trayectoria.png` mostraba muchos giros/lazos que solo se explican si esas
  paredes existen. Causa raíz en `OccupancyGridMap` (`occupancy_grid.py`): el clip
  del log-odds (`clamp=5.0`) se aplicaba en **cada** actualización, no solo al
  resultado final. Con los `p_occ`/`p_free` por defecto (0.7/0.4), eso satura una
  celda después de apenas ~6 impactos o ~12 pasadas-libres consecutivas — muy poco
  para un laberinto recorrido en varios lazos, donde una celda de pared puede
  acumular miles de impactos a lo largo de la corrida. El resultado terminaba
  dependiendo del **orden reciente** de los eventos, no de la mayoría histórica.
  Verificado directamente contra `Rosbags/corrida2_run/`: celdas con miles de
  impactos crudos (`grid.hits`, hasta 7.784 en una sola celda) quedaban con
  log-odds `-5.0` (saturado "libre"). El histograma crudo de impactos trazaba
  perfectamente todo el interior del laberinto; el mapa de probabilidad lo perdía
  casi por completo.
- [x] **Fix**: subir `clamp` de 5 a **50** (probado también 100 y 1000 contra los
  datos reales — sin mejora adicional respecto de 50, así que se usa el valor más
  moderado). Expuesto como `--clamp` en `slam_pipeline.py` (default 50).
  `entrega_parte_A/` regenerado con el fix (misma trayectoria/landmarks — el fix
  solo toca la grilla — `mapa.pgm`/`mapa.yaml`/`mapa.png` reemplazados). El rescate
  manual por impactos (`--min-hits-occ`) pasó de rescatar 30 celdas a solo 8 (el
  consenso log-odds por sí solo ya resuelve casi todo con el clamp corregido).
- [x] **`/map` en RViz para Parte A**: no existía ningún nodo publicándolo pese a
  estar documentado y en `slam.rviz` (nadie corre `map_server`, ni está instalado
  en este workspace). Nuevo `map_publisher_node` en `slam_pkg` (nodo propio, sin
  dependencias nuevas — parsea el `.pgm/.yaml` a mano) publica `/map` con QoS
  `TRANSIENT_LOCAL`; expuesto en `slam.launch.py` como arg `map_yaml_path`.
- [x] **Parte B — mapa de Gazebo vs. mapa real**: `navegacion_pkg` compartía un
  solo `map.pgm/.yaml` entre `simulation.launch.py` (Gazebo, `casa.world`, ~13×11 m)
  y `robot_real.launch.py` (robot real, laberinto, ~33×33 m) desde el commit
  `aa03121` (03/07) que estandarizó al mapa real — rompiendo la localización en
  modo Gazebo (geometría incompatible). Agregado `map_sim.pgm/.yaml` (el mapa a
  escala de `casa.world` que ya estaba en `turtlebot3_custom_simulation/worlds/map/`,
  sin usar) + parámetro `map_yaml` en `map_publisher.py` para elegir cuál cargar;
  `simulation.launch.py` ahora pasa `map_sim.yaml`, `robot_real.launch.py` sigue
  con el mapa real (default). Verificado con `ros2 topic echo /map` en ambos modos.
- [x] **Limpieza de `package.xml`/`setup.py`**: `navegacion_pkg` y `despliegue_pkg`
  no declaraban ninguna dependencia real (rclpy, numpy, scipy, PIL, tf2_ros...)
  pese a usarlas todas — completadas. Sacados los placeholders `TODO:`/`tu_nombre`
  de descripción/licencia/maintainer en 5 paquetes.

## Hecho (03/07 — pasajes angostos: fallback de planificación en el robot real)

Probando el robot real: dos obstáculos muy cercanos al inicio quedaban "pegados"
al inflar las paredes y Theta* no encontraba camino (el robot volvía a WAITING
aunque físicamente pasaba). Fix en `navegacion_pkg/robot.py` **y**
`despliegue_pkg/robot.py` (mismo planner):

- [x] **Nuevo `_reopen_narrow_passages`**: sobre el mapa inflado, reabre solo la
  **línea media** de los pasajes sellados (transformada de distancia euclidiana:
  celdas libres en el mapa real, con despeje ≥ `NARROW_REOPEN_MIN_CELLS` = 2
  celdas, que son máximo local de distancia y conectan dos regiones libres
  distintas — esto último descarta las bisectrices de esquinas, que también son
  máximos locales, y conserva el margen completo en las esquinas).
- [x] **Semántica de fallback**: se planifica siempre con el mapa inflado normal;
  solo si NO existe camino se reintenta con la variante reabierta
  (`inflated_map_reopened`, calculada en `cb_map`). Motivo: en el mapa de la
  entrega con r=4 lo único sellado es el **interior de la silla** — con
  reapertura siempre activa Theta* acortaría camino entre las patas.
  En `navegacion_pkg` el fallback reabre además *después* de proyectar los
  obstáculos dinámicos del LIDAR (`_build_planning_map(reopen_narrow=True)`);
  en `despliegue_pkg` cubre también `_run_planning_around_obstacle`.
- [x] **Verificado offline**: casos sintéticos (pared recta y esquina en L → 0
  celdas reabiertas; pasaje de 5 celdas → se reabre centrado; pasaje de 2 celdas
  impasable → sigue cerrado), smoke test de nodo completo en ambos paquetes
  (plan normal falla → WARN de fallback → WALKING con camino centrado en el
  gap) y mapa real de la entrega (r=3: 0 sellados; r=4: solo la silla,
  componentes libres 3→2). `colcon build` OK.
  Ajuste en campo: `NARROW_REOPEN_MIN_CELLS` (subir si roza, bajar si no abre).
  Artifact: https://claude.ai/code/artifact/65dd7819-7294-4f9d-85d6-030e7db06cf4

## Revisión contra la consigna (03/07) — antes del próximo turno de lab

Auditoría completa de A+B+C contra los PDFs oficiales (checklist + bugs, cada
crítico re-verificado a mano). Informe completo con severidades y evidencia
`archivo:línea`: https://claude.ai/code/artifact/c994888d-8dcf-4cb9-979a-3cb4fa75592c

**La foto**: las tres partes cumplen la arquitectura pedida; la Parte A está
completa y entregable. Los bloqueantes del robot real son bugs de cableado chicos:

1. **Offset del LIDAR inconsistente** (afecta B y C — el hallazgo más importante):
   el PF de `despliegue_pkg` proyecta el scan con **+180°**
   (`particle_filter.py:147`), el de `navegacion_pkg` con **0°** (`:164`, el `+π`
   quedó comentado tras un debug), el cono frontal de obstáculos de ambos
   `robot.py` asume **0°**, y solo `detector_cono_node.py` aplica el **+90°**
   validado (extrínseca real: `slam_pkg/occupancy_grid.py:304`). A lo sumo una
   convención es correcta → PF que no converge en el robot real + obstáculos
   frontales invisibles. Resolver con un experimento contra el bag
   (proyectar scan con 0/90/180 sobre el mapa) y unificar.
2. **Fixes de una línea**: `sigma 5.0→0.35` en `despliegue_pkg/likelihood_field.py:33`;
   `map_publisher` de ambos paquetes convierte "desconocido" (205) en libre
   (en el mapa real es el 97% del grid) → mapear 205→-1; en la sim de B nadie
   publica `/calc_odom` en `custom_casa*/` (nodo comentado) → remap a `/odom`;
   la FSM no frena al degradarse la localización (falta `Twist()` antes del
   `return`); umbral de obstáculo 0.15 m ≤ radio del TB4 (~0.17) → subir a ~0.25.
3. **TF map→odom mal compuesto en B y C**: publica la pose del robot en vez de
   `T_map_base·(T_odom_base)⁻¹` + `child_frame_id` sin namespace `tb4_0/`.
4. **Entrega (reprueban por sí solos)**: falta el **informe técnico PDF**
   (con diagrama de bloques de la FSM y apartado sim-to-real de C) y las
   **diapositivas** de la defensa (sin material visual no dejan exponer).
   Detalle B: la localización online es LIDAR-only (Sistema 1 de facto) —
   justificarlo en el informe. Detalle C: `cono_detector_pkg` anidado no se
   compila con un `colcon build` pelado — resolver para el zip de entrega.

Parte A (hallazgos menores, no bloquean): piso de bearing 0.5° optimista vs
asociación por keyframe (`graph_slam.py:412`), CSVs en modo append (mezcla
corridas al re-correr percepción), header de `scans.csv` asume haces constantes,
y falta TF map→odom + display de odom en `slam.rviz` para la demo en vivo.

## Pendiente
- [ ] **Nitidez final (opcional, agregado grande)**: para el salto final de paredes
  finas haría falta meter edges de **scan-matching dentro del GraphSLAM** (no solo en la
  2da pasada de la grilla). El fix del clamp (03/07) resuelve la omisión de
  estructura; esto sería solo un afinado adicional de nitidez.
- [ ] **Cierre de entrega**: la integración con la Parte B. (El mapa final +
  `landmarks.json` ya están empaquetados en `entrega_parte_A/`; launch files y
  config `.rviz` ya están en el repo.)
- [x] **Commitear** *(hecho 03/07)*: el fix de la silla, la Parte C, los docs y
  `entrega_parte_A/` (ahora con el mapa de la **corrida larga de 46,8 min**,
  el de `Rosbags/corrida2_run/slam_out_v2/`) están commiteados y pusheados en
  `fix-relocalizing-completo`.
- [x] **Reconciliar con `join_parts`** *(revisada 03/07 — decisión: esta branch
  queda como canónica)*: `join_parts` (de fragonesi, 02/07, desde el `main`
  viejo, layout PRA `PRA_A/B/C` + `mapa_completo/`) se revisó archivo por
  archivo. Resultado: **Parte C es byte-idéntica** a `src/parte_C/` (nada que
  traer). **Parte A**: sus cambios (odom yaw std 1°→3°, `p_occ/p_free`
  0.85/0.35 hardcodeado, PNGs de progreso, recorte `max_scan_idx`) NO se
  adoptan — nos quedamos con nuestra versión (los PNGs de progreso podrían
  portarse para el informe si hiciera falta). **Parte B**: su aporte real es la
  parametrización `robot:=tb4` dentro de los nodos (tópicos `/tb4_N/*`, QoS
  BEST_EFFORT, offset LIDAR +90°, filtro de intensidad), pero borra todos los
  docstrings; cubrimos el mismo caso de uso con `robot_real.launch.py` (abajo)
  y la adaptación real al TB4 ya vive en `src/parte_C/`.
- [x] **Parte B — fixes de PruebaB integrados** *(03/07)*: del zip `PruebaB.zip`
  (**de Zoe** — el mensaje del commit `bf02c20` dice "de Roma" por error, era
  el maintainer del setup.py) se integraron a `src/navegacion_pkg`: (1) `localization_node`
  ahora publica la **TF `map→odom`** con la media ponderada de las partículas
  (misma idea que `parte_C/localization_node_tb4.py`); (2) `map_publisher`
  vuelve a publicar **periódico a 1 Hz**; (3) **`launch/robot_real.launch.py`**
  nuevo — el stack completo remapeado a `/tb4_0/{odom,scan,cmd_vel}`;
  (4) el mapa del paquete ahora es **el de la entrega (corrida larga 46,8 min)**
  (la compañera ya usaba un mapa de esa corrida, `salida_8000`; se estandarizó
  al final). Además se pasaron los subs de odom/scan de `localization_node` y
  el scan de `robot.py` a **QoS BEST_EFFORT** (gotcha conocido: el TB4 real y
  sus bags publican BEST_EFFORT; una suscripción RELIABLE no recibe nada — el
  launch remapeado no anduvo sin esto; compatible con la sim).
- [x] **Renombrar `src/parte_C/` → `src/despliegue_pkg/`** *(03/07)*: la carpeta
  contenedora ahora matchea el nombre del paquete colcon que tiene adentro
  (`despliegue_pkg`), igual que `aruco_pkg`/`slam_pkg`/`navegacion_pkg`. Hecho
  con `git mv` (preserva historial). No hizo falta tocar código: `package.xml`,
  `setup.py` y los launch files ya referencian el paquete por nombre
  (`get_package_share_directory('despliegue_pkg')`), no por ruta de carpeta, y
  `build/`/`install/` ya estaban indexados por nombre de paquete. Verificado con
  `colcon build --packages-select despliegue_pkg` (build ok) y con
  `colcon build --paths src/despliegue_pkg/cono_detector_pkg` para el paquete
  anidado (**gotcha pre-existente, no introducido por el rename**: colcon no
  descubre `cono_detector_pkg` vía `--packages-select` porque está anidado
  dentro de la carpeta de otro paquete — hay que buildearlo con `--paths`
  explícito. El comando `colcon build --packages-select despliegue_pkg
  cono_detector_pkg --symlink-install` de `README_ParteC.md` en realidad solo
  buildeaba `despliegue_pkg` y tiraba un warning silencioso ignorando
  `cono_detector_pkg` — **corregido** en la sección Build de ese README: ahora
  son dos comandos, uno por paquete, el segundo con `--paths` explícito;
  probado con `--symlink-install` y funciona). Las menciones
  a `src/parte_C/` y `parte_C/...` más arriba son históricas y se refieren a
  esta misma carpeta con su nombre viejo.

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
