# TP Final — Principios de la Robótica Autónoma (I-402, UdeSA)

**Opción 3 — Features con Cámara (ArUco + Graph SLAM).**
Robot TurtleBot4 en un laberinto. El trabajo tiene tres partes:

- **Parte A — Percepción + SLAM.** Construir un mapa del entorno estimando a la vez
  la pose del robot, con **Graph SLAM** (odometría + observaciones ArUco rango-bearing
  + cierre de lazo por re-observación de ID) y una segunda pasada de LIDAR que proyecta
  los barridos sobre la trayectoria corregida para generar la **grilla de ocupación**.
- **Parte B — Navegación.** Localización por filtro de partículas + planificación
  (Theta*) + seguimiento (Pure Pursuit) sobre el mapa de la Parte A, en simulación
  (Gazebo, TurtleBot3).
- **Parte C — Despliegue en robot real.** La misma pila de navegación adaptada al
  **TurtleBot4 real**, con exploración autónoma y detección de conos rojos por cámara.

Todo el código, comentarios y documentación están en **español**. Entorno:
**ROS 2 Humble**, Python, paquetes `ament_python`.

---

## Estructura del repositorio

Cada parte es un **workspace colcon independiente** (se compila por separado desde
su carpeta):

```
TP_Final_Robotica/
├── src/
│   ├── aruco_pkg/             ← detección ArUco, pose por solvePnP, odometría (deltas), modelo de ruido
│   ├── slam_pkg/              ← Graph SLAM (batch LM + keyframes + loop closure) y grilla de ocupación
│   ├── navegacion_pkg/       ← FSM + Theta* + Pure Pursuit + filtro de partículas
│   ├── deploy_pkg/           ← FSM extendida + localización adaptada al TB4
│   └── cono_detector_pkg/   ← detector de conos rojos (HSV + fusión LIDAR)
├── entrega_parte_A/          ← ENTREGABLES de la Parte A (mapa, landmarks, trayectoria)
├── Informe_TP_Final_Robotica.pdf  ← informe técnico (documenta A, B y C)
└── README.md                      ← este archivo
```

### Entregables

- **Parte A** produce artefactos concretos, en `PRA_A/entrega_parte_A/`: la **grilla de
  ocupación** (`mapa.pgm`/`mapa.yaml`) y los **hitos ArUco por ID** (`landmarks.json`).
- **Partes B y C** no generan un archivo entregable: se evalúan por el **comportamiento**
  del sistema (localización, planificación, seguridad, exploración y guiado hacia el
  cono). Su documentación —incluido el análisis obligatorio de la **brecha sim-to-real**
  de la Parte C— va en el **`Informe_TP_Final_Robotica.pdf`**.

> Los directorios `build/`, `install/`, `log/` y `__pycache__/` son artefactos de
> compilación: no se versionan (`.gitignore`) y se regeneran con `colcon build`.

---

## Requisitos

- **ROS 2 Humble** (`/opt/ros/humble`)
- Python 3 con: `numpy`, `scipy`, `opencv-python`, `cv_bridge`, `Pillow`
- Los rosbags del laberinto (no están en el repo por tamaño).

En **cada terminal** sourcear ROS antes de trabajar:

```bash
source /opt/ros/humble/setup.bash
```

---

## Parte A — Percepción + SLAM

### Build

```bash
cd PRA_A
colcon build --symlink-install
source install/setup.bash
```

### 1) Percepción → CSVs

En una terminal reproducir el bag; en otra, correr la percepción:

```bash
# terminal 1
ros2 bag play /ruta/al/bag/laberinto

# terminal 2  (dict DICT_4X4_50 obligatorio, ver "Notas")
ros2 launch slam_pkg perception.launch.py
```

Salida: `odom_deltas.csv`, `<bag>_detections.csv` y (para la 2da pasada) `scans.csv`.

### 2) Graph SLAM → entregables

`slam_pipeline.py` corre como script suelto (imports planos), así que se ejecuta
**desde dentro del módulo** (`PRA_A/src/slam_pkg/slam_pkg/`):

```bash
cd src/slam_pkg/slam_pkg
python3 slam_pipeline.py \
    --odom  odom_deltas.csv \
    --aruco laberinto_detections.csv \
    --scans scans.csv \
    --noise-model ../../aruco_pkg/aruco_pkg/noise_model.json \
    --sm-passes 3 --min-hits-occ 500 \
    --out-dir salida
```

Genera `mapa.pgm`/`mapa.yaml`, `landmarks.json`, `trayectoria_opt.csv` y PNGs de
diagnóstico. `--clamp` (default 50) controla el clip del log-odds de la grilla;
`--min-hits-occ` rescata obstáculos finos (patas de silla) que el consenso borraría.

### Visualización en RViz

```bash
ros2 launch slam_pkg slam.launch.py \
    odom_csv:=/ruta/odom_deltas.csv aruco_csv:=/ruta/laberinto_detections.csv
```

Tópicos: `tb4_0/scan`, `tb4_0/odom`, `/belief` (trayectoria corregida),
`/landmarks` (ArUcos en el marco del mapa), `/poses_guardadas` (nodos del grafo),
`/map` (grilla de ocupación).

### Entregables (en `PRA_A/entrega_parte_A/`)

Generados sobre el bag de la corrida larga (46,8 min, múltiples lazos): **46 landmarks**,
χ² 54.442 → 1.686.

| Archivo | Qué es |
|---|---|
| `mapa.pgm` / `mapa.yaml` | Grilla de ocupación (`0`=ocupado, `254`=libre, `205`=desconocido), 0.05 m/celda, lista para A*/Dijkstra en B y C |
| `landmarks.json` | Hitos ArUco por ID en el marco del mapa: `{id: [x, y]}` |
| `trayectoria_opt.csv` | Trayectoria corregida por keyframe (`x, y, theta`) |
| `mapa.png` / `trayectoria.png` | Vistas de diagnóstico |

---

## Parte B — Navegación (simulación TB3)

### Build

```bash
cd PRA_B
colcon build --symlink-install
source install/setup.bash
```

### Correr

```bash
# Simulación (Gazebo + TurtleBot3)
ros2 launch navegacion_pkg simulation.launch.py

# Sobre robot / bag real (tópicos remapeados a /tb4_0/*)
ros2 launch navegacion_pkg robot_real.launch.py
```

En RViz, fijar **Fixed Frame = `map`** y dar un **2D Pose Estimate** para inicializar
el filtro de partículas; recién cuando la nube converge el robot empieza a navegar.

Nodos: `robot_node`, `localization_node`, `likelihood_field_node`, `map_publisher`.

---

## Parte C — Despliegue en TurtleBot4 real

El robot explora el laberinto buscando conos rojos; al detectar uno planifica una
ruta evitando paredes y navega hasta él, y luego vuelve a explorar.

```
WAITING → EXPLORING → CONE_DETECTED → PLANNING → WALKING → ALIGNING → EXPLORING → …
```

### Build

```bash
cd PRA_C
colcon build --symlink-install     # compila deploy_pkg + cono_detector_pkg
source install/setup.bash
```

### Correr (el robot real publica sus tópicos automáticamente)

```bash
# terminal 1 — pila de navegación (abre RViz)
ros2 launch deploy_pkg robot_real.launch.py

# terminal 2 — detector de conos
ros2 run cono_detector_pkg detector_cono

# terminal 3 — (validación con bag, opcional)
ros2 bag play /ruta/al/bag/laberinto_conos
```

Igual que en la Parte B: **Fixed Frame = `map`** + **2D Pose Estimate** para converger
el filtro. Parámetros ajustables (velocidades de exploración, `CONE_GOAL_OFFSET`,
radio de inflación) en `deploy_pkg/robot_parte_c.py` y `deploy_pkg/robot.py`.

Nodos: `robot_node_c`, `localization_node_tb4`, `map_publisher`, `detector_cono`.

Umbrales HSV del detector (`cono_detector_pkg/detector_cono_node.py`) calibrados con el
bag `laberinto_conos`; reajustar según la iluminación del laboratorio. Detalle del
comportamiento y del análisis sim-to-real, en el informe.

---

## Notas que evitan resultados en cero

1. **Diccionario ArUco = `DICT_4X4_50`.** El default del detector (`DICT_5X5_250`)
   detecta candidatos pero decodifica **cero** IDs. Pasar
   `-p aruco_dictionary:=DICT_4X4_50`.
2. **QoS `BEST_EFFORT` en las suscripciones a sensores.** El bag y el TB4 publican
   `odom`/`scan` con `BEST_EFFORT`; una suscripción `RELIABLE` (default de ROS 2) no
   recibe **ningún** mensaje (solo un warning de QoS incompatible).
3. **Offset del LIDAR.** El rplidar del TB4 está rotado **+90°** respecto de
   `base_link`; sin esa corrección las paredes salen borrosas / el cono frontal queda
   girado.
4. La cámara del TB4 solo expone la imagen `preview` de baja resolución: el detector
   la **reescala 4× (Lanczos)** y escala `K` por el mismo factor.
