# TP Final — Robótica Autónoma · Parte A (Opción 3: *Features con Cámara*)

SLAM offline para un **TurtleBot4** en un laberinto: se construye un mapa del
entorno mientras se estima la pose del robot, todo **contra rosbags pre-grabados**
(no hay robot en vivo ni *ground truth*). El mapa resultante es la base de las
Partes B y C. Código, comentarios y documentación en **español**.

> **Materia:** I-402 — Principios de la Robótica Autónoma (UdeSA).
> **Entrega:** Parte A únicamente. Opción elegida: **3 (Features con Cámara)**.

---

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

## Estructura del repo

Workspace `colcon`/`ament_python` con **dos paquetes** bajo `src/`:

```
TP_Final_Robotica/                 ← raíz del workspace (acá se corre colcon)
├── README.md                      ← este archivo
├── ESTADO.md                      ← bitácora de avances / handoff
├── TpParteA.md                    ← log detallado de decisiones, bugs y soluciones
└── src/
    ├── aruco_pkg/                 ← PERCEPCIÓN + odometría + modelo de ruido
    │   ├── aruco_pkg/
    │   │   ├── aruco_detector_node.py   detección ArUco + pose 3D (solvePnP)
    │   │   ├── odom_delta_node.py        odometría → deltas (rot1, trans, rot2)
    │   │   ├── aruco_noise_model.py      covarianza dependiente de la distancia
    │   │   └── aruco_dict_probe.py       (debug) prueba ~20 diccionarios ArUco
    │   ├── scripts/                      fit_noise_model.py · plot_odom.py
    │   └── config/                       calibracion_ejemplo.yaml
    └── TP_Final_Robotica/         ← GRAPH SLAM (back-end)
        ├── TP_Final_Robotica/
        │   ├── graph_slam.py             núcleo: mínimos cuadrados disperso (LM)
        │   ├── slam_pipeline.py          "corré-todo": CSV → entregables
        │   ├── graph_slam_node.py        nodo ROS para RViz (/belief, /landmarks…)
        │   ├── occupancy_grid.py         2da pasada: grilla de ocupación (.pgm/.yaml)
        │   ├── scan_logger_node.py       registra los LaserScan a CSV
        │   └── gslam.py                  (borrador de referencia, NO ejecutable)
        ├── launch/                       slam.launch.py · perception.launch.py
        └── rviz/                         slam.rviz
```

> Los rosbags y los CSV/PNG generados **no van a git** (pesan demasiado; ver
> `.gitignore`). El bag del laberinto (~8 GB) se descarga aparte.

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
ros2 launch TP_Final_Robotica perception.launch.py

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
ros2 run TP_Final_Robotica scan_logger_node --ros-args \
    -p scan_topic:=/tb4_0/scan -p log_csv_path:=scans.csv
```
</details>

### Paso 2 — GraphSLAM: de los CSV a los entregables

El pipeline corre como script suelto (no necesita ROS). Desde
`src/TP_Final_Robotica/TP_Final_Robotica/`:

```bash
python3 slam_pipeline.py \
    --odom  odom_deltas.csv \
    --aruco laberinto_detections.csv \
    --scans scans.csv \
    --noise-model ../../aruco_pkg/aruco_pkg/noise_model.json \
    --out-dir salida
```

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
ros2 launch TP_Final_Robotica slam.launch.py \
    odom_csv:=/ruta/odom_deltas.csv \
    aruco_csv:=/ruta/laberinto_detections.csv
```

Publica en los **tópicos canónicos** que esperan los profes:

| Tópico | Tipo | Contenido |
|---|---|---|
| `/belief`          | `nav_msgs/Path`        | trayectoria corregida |
| `/landmarks`       | `MarkerArray`          | ArUcos estimados, por ID |
| `/poses_guardadas` | `geometry_msgs/PoseArray` | nodos de pose del grafo |
| `/map`             | `nav_msgs/OccupancyGrid` | grilla de ocupación (via `map_server`) |

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
cd src/TP_Final_Robotica/TP_Final_Robotica
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
  sobre el bag real (50 landmarks coherentes; χ² baja con gating).
- ✅ **Trayectoria corregida** y **landmarks por ID**: entregables listos.
- ✅ **Modelo de ruido ArUco (`std = a + b·d`) enchufado en GraphSLAM**: covarianza
  de cada observación sale del fit real (propagada de cartesiano a rango-bearing),
  con término sistemático de escala para evitar sobre-confianza en el rango.
- 🟡 **Grilla de ocupación**: se genera y exporta. Afinada (29/06) con **cap de rango**
  (~5 m, saca los chorros espurios), **más cobertura de barridos** (4000), **filtrado por
  intensidad** (`scan_logger` guarda `i0..iN`) y **scan-matching contra el mapa** (estilo
  Hector: `LikelihoodField` + Gauss-Newton por barrido, flag `--sm-passes`) que adelgaza
  el borrón de las paredes (error de pose). El salto final de nitidez requeriría edges de
  scan-matching dentro del GraphSLAM (pendiente, agregado mayor). Ver `TpParteA.md`.

> Bitácoras detalladas: **`ESTADO.md`** (avances/handoff) y **`TpParteA.md`**
> (decisiones, bugs y cómo se resolvieron). Leerlas antes de extender la Parte A.
