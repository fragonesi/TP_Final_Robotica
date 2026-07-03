# TP Final — Robótica Autónoma (Partes A, B y C)

Trabajo Práctico Final completo: **Parte A** (SLAM offline contra rosbags del
TurtleBot4 real, Opción 3 — *Features con Cámara*), **Parte B** (navegación
autónoma en simulación Gazebo, con un modo adicional para correr el mismo
stack contra el robot real) y **Parte C** (despliegue en el TurtleBot4 físico +
misión de búsqueda de conos rojos). Código, comentarios y documentación en
**español**.

> **Materia:** I-402 — Principios de la Robótica Autónoma (UdeSA).
> **Opción elegida en Parte A:** 3 (Features con Cámara) → determina que en
> Parte B se use "Sistema 3: Grilla + Landmarks de Cámara" (ver más abajo).

Este README cubre las tres partes de punta a punta: **cómo generar el mapa**
(Parte A), **cómo correr la navegación** (Parte B, en Gazebo o contra el robot
real) y **cómo desplegar en el TurtleBot4 físico** (Parte C). Cada parte tiene
también su propia documentación más detallada:
- Parte A: este mismo archivo (abajo) + `TpParteA.md` (bitácora de decisiones/bugs).
- Parte C: `src/despliegue_pkg/README_ParteC.md` (guía completa paso a paso).

---

## Estructura del repo

Workspace `colcon`/`ament_python` con **paquetes** bajo `src/`:

```
TP_Final_Robotica/                 ← raíz del workspace (acá se corre colcon)
├── README.md                      ← este archivo
├── ESTADO.md                      ← bitácora de avances / handoff
├── TpParteA.md                    ← log detallado de decisiones, bugs y soluciones (Parte A)
├── entrega_parte_A/                ← entregables finales de Parte A (mapa + landmarks)
└── src/
    ├── aruco_pkg/                 ← Parte A: percepción ArUco + odometría + modelo de ruido
    ├── slam_pkg/                  ← Parte A: GraphSLAM (back-end) + grilla de ocupación
    ├── navegacion_pkg/             ← Parte B: localización (filtro de partículas) +
    │                                  Theta* + Pure Pursuit + máquina de estados
    ├── aruco_sim_msgs/             ← Parte B: mensaje custom para el sensor virtual de ArUco
    ├── aruco_sim_pkg/              ← Parte B (Sistema 3): sensor virtual de landmarks para
    │                                  Gazebo (densidad + oclusión por línea de visión)
    ├── turtlebot3_custom_simulation/ ← mundos y launch files de Gazebo (`custom_casa*.launch.py`)
    └── despliegue_pkg/             ← Parte C: despliegue en TurtleBot4 real
        └── cono_detector_pkg/      ← detector de conos rojos (paquete anidado, ver Build)
```

> Los rosbags y los CSV/PNG generados **no van a git** (pesan demasiado; ver
> `.gitignore`).

---

# Parte A — SLAM (Opción 3: Features con Cámara)

SLAM offline para un **TurtleBot4** en un laberinto: se construye un mapa del
entorno mientras se estima la pose del robot, todo **contra rosbags pre-grabados**
(no hay robot en vivo ni *ground truth*). El mapa resultante es la base de las
Partes B y C.

## Qué entrega esta parte (los dos entregables)

1. **Mapa de grilla de ocupación** métrico, exportado en formato `map_server`
   (`mapa.pgm` + `mapa.yaml`), apto para un planificador global (A*/Dijkstra).
2. **Landmarks por ID de ArUco** (`landmarks.json`): `{marker_id: [x, y]}` en el
   frame del mapa.

Lo exige la consigna usando **Graph SLAM con cierre de lazo** (obligatorio, no
EKF/FastSLAM) y un procedimiento de **dos pasadas**:

1. Optimizar el grafo de poses con **odometría + ArUcos** → trayectoria corregida
   + landmarks.
2. Reproyectar el **LIDAR** sobre esa trayectoria corregida → grilla de ocupación.

El cierre de lazo sale "gratis": cuando el robot vuelve a ver un ArUco ya visto, su
**ID da la asociación de datos** y esa observación ata el grafo, corrigiendo la
deriva acumulada (la única corrección posible sin *ground truth*).

---

## ⚠️ Dos gotchas que silenciosamente producen CERO resultados

Antes de correr nada, tener presente:

1. **El diccionario ArUco debe ser `DICT_4X4_50`.** El default del detector es
   `DICT_5X5_250`, que detecta *candidatos* (cuadrados) pero decodifica **cero**
   IDs contra los marcadores reales. Siempre pasar `-p aruco_dictionary:=DICT_4X4_50`.
   (`aruco_dict_probe.py` fuerza-bruta ~20 diccionarios para confirmarlo.)

2. **Las suscripciones a sensores del bag deben usar QoS `BEST_EFFORT`.** El bag
   publica `/tb4_0/odom` y `/tb4_0/scan` como `BEST_EFFORT`; la suscripción default
   de ROS 2 es `RELIABLE`, y el mismatch hace que **no se entregue ningún mensaje**
   (solo un warning de "QoS incompatible"). `odom_delta_node` y `scan_logger_node`
   ya lo declaran explícito.

---

## Requisitos

- **ROS 2 Humble** (`/opt/ros/humble`).
- Python 3 con **numpy, scipy, pandas, matplotlib** (declarados en `package.xml`).
- **OpenCV** + `cv_bridge` (el detector soporta la API nueva ≥4.7 y la vieja ≤4.6).

---

## Build

Desde la raíz del workspace (este directorio):

```bash
colcon build
source install/setup.bash        # necesario para que `ros2 run`/`ros2 launch` vean los nodos
```

> Cada nodo nuevo se registra en el `setup.py` de su paquete como *entry point* y
> recién se ve por `ros2 run` después de re-compilar.

---

## Flujo end-to-end (de cero al mapa)

### Paso 0 — Conseguir el bag del laberinto

El bag pesa ~8 GB y no está en el repo. Descargarlo y dejar la carpeta a mano.

### Paso 1 — Percepción: del bag a los CSV

Levantar los nodos de percepción y, **en otra terminal**, reproducir el bag:

```bash
# Terminal A: nodos de percepción (detector ArUco + odometría + logger de LIDAR)
ros2 launch slam_pkg perception.launch.py

# Terminal B: reproducir el bag (el laberinto dura ~23 min)
ros2 bag play /ruta/a/laberinto
```

Esto genera tres CSV en el directorio de trabajo:
`laberinto_detections.csv`, `odom_deltas.csv` y `scans.csv`.

<details>
<summary>Equivalente corriendo cada nodo a mano (sin launch)</summary>

```bash
# Detector ArUco — ¡con el override del diccionario!
ros2 run aruco_pkg aruco_detector_node --ros-args \
    -p image_topic:=/tb4_0/oakd/rgb/preview/image_raw \
    -p camera_info_topic:=/tb4_0/oakd/rgb/preview/camera_info \
    -p marker_length:=0.15 -p upscale_factor:=4.0 \
    -p use_clahe:=false -p aruco_dictionary:=DICT_4X4_50 \
    -p log_csv_path:=laberinto_detections.csv

# Odometría → deltas
ros2 run aruco_pkg odom_delta_node --ros-args \
    -p odom_topic:=/tb4_0/odom -p log_csv_path:=odom_deltas.csv

# LIDAR → CSV (para la 2da pasada)
ros2 run slam_pkg scan_logger_node --ros-args \
    -p scan_topic:=/tb4_0/scan -p log_csv_path:=scans.csv
```
</details>

### Paso 2 — GraphSLAM: de los CSV a los entregables

El pipeline corre como script suelto (no necesita ROS). Desde
`src/slam_pkg/slam_pkg/`:

```bash
python3 slam_pipeline.py \
    --odom  odom_deltas.csv \
    --aruco laberinto_detections.csv \
    --scans scans.csv \
    --noise-model ../../aruco_pkg/aruco_pkg/noise_model.json \
    --sm-passes 3 --max-scans 19926 --min-hits-occ 500 \
    --out-dir salida
```

Flags útiles de la grilla (2da pasada):

- `--max-scans N`: barridos a usar (default 4000; pasar el total del CSV para usar todos).
- `--sm-passes N`: pasadas de scan-matching coarse-to-fine (el entregable usó 3).
- `--min-hits-occ N`: **rescate de obstáculos finos** — celdas con ≥N impactos crudos
  y lejos de paredes consensuadas se exportan ocupadas aunque el log-odds las marque
  libres. Sin esto, una pata de silla (~2 cm en celdas de 5 cm) desaparece del mapa:
  los rayos que pasan al lado la "liberan" más de lo que los impactos la ocupan.
  Escalar N con `--max-scans` (500 con ~20k scans ≈ 2,5%). Ver `TpParteA.md` (02/07).
- `--p-occ / --p-free`: modelo inverso del sensor del log-odds (defaults 0.7/0.4).

Genera en `salida/`:

| Archivo | Qué es |
|---|---|
| `landmarks.json`      | **ENTREGABLE** — landmarks ArUco por ID `{id: [x,y]}` |
| `mapa.pgm` + `mapa.yaml` | **ENTREGABLE** — grilla de ocupación (formato `map_server`) |
| `trayectoria_opt.csv` | trayectoria corregida (x, y, θ por keyframe) |
| `trayectoria.png`     | diagnóstico: odometría cruda vs. corregida + landmarks |
| `mapa.png`            | diagnóstico: render de la grilla |

> Sin `--aruco` el grafo solo usa odometría (no hay cierre de lazo). Sin `--scans`
> no se genera la grilla (solo trayectoria + landmarks).

### Paso 3 — Visualizar en RViz

```bash
ros2 launch slam_pkg slam.launch.py \
    odom_csv:=/ruta/odom_deltas.csv \
    aruco_csv:=/ruta/laberinto_detections.csv \
    map_yaml_path:=/ruta/a/salida/mapa.yaml
```

`map_yaml_path` es opcional (default vacío = no publica `/map`, el resto de
RViz funciona igual); apunta al `mapa.yaml` que generó `slam_pipeline.py` en
el Paso 2 (p. ej. `entrega_parte_A/mapa.yaml` para ver el mapa final ya
entregado).

Publica en los **tópicos canónicos** que esperan los profes:

| Tópico | Tipo | Contenido |
|---|---|---|
| `/belief`          | `nav_msgs/Path`        | trayectoria corregida |
| `/landmarks`       | `MarkerArray`          | ArUcos estimados, por ID |
| `/poses_guardadas` | `geometry_msgs/PoseArray` | nodos de pose del grafo |
| `/map`             | `nav_msgs/OccupancyGrid` | grilla de ocupación, leída del `.pgm/.yaml` por `map_publisher_node` (nodo propio, liviano — no depende de `nav2_map_server`, que no está instalado en este workspace) |

---

## Cómo funciona (resumen para el informe)

**Front-end (percepción).**
- `aruco_detector_node`: detecta ArUcos y estima su pose 3D con `solvePnP`
  (`IPPE_SQUARE`). Como el único tópico de imagen es el `preview` de baja
  resolución, agranda 4× (Lanczos) y escala `K` por el mismo factor.
- `odom_delta_node`: convierte la odometría al modelo de Thrun
  `(δrot1, δtrans, δrot2)`, con guard para no inyectar ruido en giros in-place.
- `fit_noise_model.py` / `NoiseModel`: ajusta `std(d) = a + b·d` por ventanas
  deslizantes con *detrend* → covarianza que pondera cada observación ArUco.

**Back-end (`graph_slam.py`).** GraphSLAM 2D **batch** por mínimos cuadrados no
lineales (equivalente a la forma de información `Ω = JᵀΩ_zJ` de Thrun, pero
iterando y re-linealizando):
- **Edges de odometría** entre keyframes (medición relativa SE(2)).
- **Edges de observación** rango-bearing por `(keyframe, landmark)`, con
  covarianza creciente con la distancia.
- Resuelve con **Levenberg-Marquardt disperso** (scipy) sobre **keyframes** →
  escala a las ~28k lecturas del laberinto sin g2o/GTSAM.
- **Gating de outliers** (Mahalanobis χ², estilo EKF) descarta detecciones ArUco
  espurias y re-optimiza.

**Segunda pasada (`occupancy_grid.py`).** Modelo de mapeo con poses conocidas
(log-odds + Bresenham): cada haz del LIDAR libera las celdas que atraviesa y ocupa
la del impacto. La pose de cada barrido se **interpola por timestamp** sobre la
trayectoria corregida. Atención a las **extrínsecas** del TurtleBot4 (el rplidar
está rotado +90° respecto de `base_link`): sin eso las paredes salen borrosas.

---

## Autotests (sin necesidad del bag)

Ambos módulos núcleo se autovalidan con datos sintéticos:

```bash
cd src/slam_pkg/slam_pkg
python3 graph_slam.py        # lazo cuadrado: RMSE odom 0.45 m → 0.037 m tras SLAM ✓
python3 occupancy_grid.py    # reconstruye una sala: 100% paredes, 100% interior libre ✓
```

Tests de linters de ament:

```bash
colcon test && colcon test-result --verbose
```

---

## Estado y limitaciones conocidas

- ✅ **Percepción, odometría, GraphSLAM y cierre de lazo**: completos y verificados
  sobre el bag real (46-50 landmarks coherentes; χ² baja con gating).
- ✅ **Trayectoria corregida** y **landmarks por ID**: entregables listos
  (`entrega_parte_A/`).
- ✅ **Modelo de ruido ArUco (`std = a + b·d`) enchufado en GraphSLAM**: covarianza
  de cada observación sale del fit real (propagada de cartesiano a rango-bearing),
  con término sistemático de escala para evitar sobre-confianza en el rango.
- ✅ **`/map` en RViz** (03/07): `map_publisher_node` (nuevo, `slam_pkg`) publica el
  `.pgm/.yaml` exportado como `nav_msgs/OccupancyGrid` con QoS `TRANSIENT_LOCAL` —
  antes el tópico simplemente no existía pese a estar documentado y en `slam.rviz`.
- ✅ **Paredes internas del laberinto recuperadas** (03/07, `entrega_parte_A/`
  regenerado): el `mapa.pgm` capturaba bien el perímetro externo pero casi ninguna
  pared interna, pese a que `trayectoria.png` mostraba un recorrido con muchos giros
  que solo se explican si esas paredes existen. Causa raíz: `OccupancyGridMap`
  (`occupancy_grid.py`) clippeaba el log-odds a `clamp=5.0` en **cada** actualización
  (no solo al final); con los `p_occ`/`p_free` por defecto eso satura una celda
  después de ~6 impactos o ~12 pasadas-libres consecutivas, muy poco para un
  laberinto recorrido en varios lazos — el resultado terminaba dependiendo del
  *orden reciente* de los eventos, no de la mayoría histórica (una celda con miles
  de impactos reales podía quedar marcada libre solo por las últimas pasadas antes
  de que el robot se alejara). Verificado contra los datos reales: el histograma
  crudo de impactos (`grid.hits`) trazaba todo el interior del laberinto con
  claridad; el mapa de probabilidad lo perdía casi por completo. Subir `clamp` a
  **50** (nuevo default, expuesto como `--clamp` en `slam_pipeline.py`; probado
  también 100 y 1000 sin mejora adicional) resuelve el problema. Detalle completo
  en `entrega_parte_A/README.md` y `TpParteA.md` (03/07).

> Bitácoras detalladas: **`ESTADO.md`** (avances/handoff) y **`TpParteA.md`**
> (decisiones, bugs y cómo se resolvieron). Leerlas antes de extender la Parte A.

---

# Parte B — Navegación Autónoma (Sistema 3: Grilla + Landmarks de Cámara)

El robot navega punto a punto sobre el mapa de la Parte A: localización por
**filtro de partículas** (odometría + LIDAR + *likelihood field*), planificación
con **Theta\*** sobre la grilla inflada, seguimiento con **Pure Pursuit**, alineación
al ángulo final, replanificación ante nueva `goal_pose` o ante obstáculos no
mapeados, y una **máquina de estados** (`WAITING → PLANNING → WALKING →
AVOIDING/ALIGNING`) que gobierna todo. Paquete: `src/navegacion_pkg/`.

Como la Parte A usó cámara (Opción 3), acá corresponde el **Sistema 3**: dado que
Gazebo no tiene marcadores ArUco nativos, `src/aruco_sim_pkg/` implementa un
**sensor virtual de landmarks** (`virtual_aruco_sensor_node`) que emula la densidad
real (~1.4–1.6 landmarks/m², comparable a los ~50 ArUcos reales de la Parte A) y
calcula **oclusión por línea de visión** contra paredes y muebles del mundo Gazebo
(`src/aruco_sim_pkg/aruco_sim_pkg/occlusion.py`) — si algo se interpone, esa
lectura no se publica.

## Build

```bash
colcon build --packages-select navegacion_pkg aruco_sim_pkg aruco_sim_msgs \
    turtlebot3_custom_simulation --symlink-install
source install/setup.bash
```

## Cómo correr en simulación (Gazebo)

```bash
ros2 launch navegacion_pkg simulation.launch.py
```

Esto levanta Gazebo con `custom_casa.launch.py` (mundo `casa.world`), más
`likelihood_field_node`, `localization_node` (filtro de partículas),
`robot_node` (máquina de estados) y `map_publisher` (publica `/map`), y abre
RViz. La consigna también pide probar contra `custom_casa_obs.launch.py`
(mundo con obstáculos) — como `simulation.launch.py` referencia
`custom_casa.launch.py` directo en el código (no hay un launch-argument para
elegir el mundo), para probarlo hay que editar esa línea en
`src/navegacion_pkg/launch/simulation.launch.py` para que apunte a
`custom_casa_obs.launch.py` y volver a `colcon build`. **No existe** todavía
`custom_casa_obs2.launch.py` (el mundo con más obstáculos y rutas cerradas que
pide la consigna como desafío **opcional**).

En RViz:
1. **2D Pose Estimate** → fija la pose inicial (tópico `initialpose`); esperar a
   que la nube de partículas converja.
2. **2D Goal Pose** → fija el objetivo (tópico `goal_pose`); el robot planifica y
   se mueve solo. Se puede repetir o cambiar de objetivo en cualquier momento
   (el robot replanifica).

> **`navegacion_pkg` usa dos mapas distintos según el modo** (arreglado 03/07 —
> antes ambos modos compartían el mapa real de la Parte A, que no corresponde a
> `casa.world`): `map_sim.pgm/.yaml` (~13×11 m, la misma escala y forma que
> `casa.world`, con los muebles del mundo) para `simulation.launch.py`, y
> `map.pgm/.yaml` (el mapa real de `entrega_parte_A/`, ~33×33 m) para
> `robot_real.launch.py`. Lo selecciona el parámetro `map_yaml` del nodo
> `map_publisher` (`simulation.launch.py` ya lo pasa; no hace falta tocar nada
> a mano).

## Cómo correr contra el robot real (sin la misión de conos)

Para probar solo navegación punto a punto en el TurtleBot4 físico (sin la lógica
de conos de la Parte C):

```bash
ros2 launch navegacion_pkg robot_real.launch.py
```

Remapea el mismo stack a `/tb4_0/{odom,scan,cmd_vel}` con QoS `BEST_EFFORT`
(gotcha #2 de más arriba, aplicado también acá). Corre sobre el mapa real de
`entrega_parte_A/` (`map.pgm/.yaml`, el default de `map_publisher`) — acá sí es
el mapa correcto, porque el robot real está en el laberinto real. Seguir los
mismos pasos de RViz (2D Pose Estimate, 2D Goal Pose) que en simulación.

---

# Parte C — Despliegue en TurtleBot4 real + búsqueda de conos rojos

El robot explora el laberinto real de forma autónoma y navega hacia conos
rojos cuando los detecta (ignorando conos de otros colores), evitando choques
con paredes aunque vea un cono a través de una abertura. Paquete:
`src/despliegue_pkg/` (+ `cono_detector_pkg/`, anidado).

**Guía completa paso a paso:** `src/despliegue_pkg/README_ParteC.md` (build,
las 4 terminales para correrlo, cómo hacer el 2D Pose Estimate, tabla de
parámetros ajustables, topics). Resumen:

```bash
# Build (dos comandos: cono_detector_pkg está anidado y colcon no lo autodetecta)
colcon build --packages-select despliegue_pkg --symlink-install
colcon build --paths src/despliegue_pkg/cono_detector_pkg --symlink-install
source install/setup.bash

# Terminal 1 — navegación + mapa + localización
ros2 launch despliegue_pkg robot_real.launch.py

# Terminal 2 — detector de conos (corre aparte, no lo levanta el launch de arriba)
ros2 run cono_detector_pkg detector_cono

# Terminal 3 — robot real (no hace falta correr nada) o, para validar antes del
# laboratorio, un rosbag de prueba:
ros2 bag play /ruta/al/bag/laberinto_conos
```

Después, en RViz: **2D Pose Estimate** sobre la posición real del robot en el
mapa (obligatorio cada vez que se arranca el sistema) y esperar a que el
filtro de partículas converja — recién ahí el robot pasa a explorar solo.

Cómo llega una detección de cono a un movimiento del robot: el detector
(`cono_detector_pkg`) segmenta rojo por HSV (dos rangos, cruza el 0/180 del
Hue) + filtro de área mínima, funde con el LIDAR para obtener rango, y publica
un punto en frame del robot. La máquina de estados de Parte C
(`robot_parte_c.py`) lo convierte a coordenadas del mapa y se lo pasa a
**Theta\*** (heredado de Parte B) para planificar — el requisito de "no
atravesar paredes caladas" lo cumple el planificador por diseño (nunca traza
un camino a través de una celda ocupada del mapa de la Parte A), no una
validación de línea de visión aparte en el detector.
