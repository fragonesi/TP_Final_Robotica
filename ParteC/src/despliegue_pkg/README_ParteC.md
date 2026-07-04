# TP Final — Parte C: Despliegue en TurtleBot4 Real

**Materia:** I-402 — Principios de la Robótica Autónoma (UdeSA)
**Opción:** 3 — Features con Cámara (ArUco + Graph SLAM)

---

## Qué hace esta parte

El robot TurtleBot4 explora autónomamente un laberinto real buscando conos rojos.
Cuando detecta uno, planifica una ruta evitando paredes y navega hasta él.
Al llegar, vuelve a explorar buscando más conos.

### Pipeline completo

```
WAITING → EXPLORING → CONE_DETECTED → PLANNING → WALKING → ALIGNING → EXPLORING → ...
```

- **WAITING**: espera que el filtro de partículas converja tras el 2D Pose Estimate
- **EXPLORING**: navegación reactiva por el laberinto (avanza, gira si hay pared)
- **CONE_DETECTED**: convierte la posición del cono al frame del mapa
- **PLANNING**: Theta* calcula el camino evitando paredes (heredado de Parte B)
- **WALKING**: Pure Pursuit sigue el camino (heredado de Parte B)
- **ALIGNING**: alineación final al llegar al cono (heredado de Parte B)

---

## Estructura del paquete

```
src/despliegue_pkg/                 ← (antes `src/parte_C/`, renombrado 03/07 para que la
│                                       carpeta contenedora matchee el nombre del paquete
│                                       colcon que tiene adentro, igual que aruco_pkg/slam_pkg/navegacion_pkg)
├── cono_detector_pkg/              ← detector de conos rojos (NUEVO Parte C)
│   └── cono_detector_pkg/
│       └── detector_cono_node.py  ← segmentación HSV + fusión LIDAR
├── launch/
│   ├── robot_real.launch.py       ← launch para robot real (NUEVO Parte C)
│   └── simulation.launch.py       ← launch original Parte B (Gazebo TB3)
├── despliegue_pkg/                 ← (antes `tpf`, renombrado para consistencia con aruco_pkg/slam_pkg/navegacion_pkg)
│   ├── robot_parte_c.py           ← FSM extendida (NUEVO Parte C)
│   ├── localization_node_tb4.py   ← localización adaptada TB4 (NUEVO Parte C)
│   ├── robot.py                   ← FSM base (Parte B, no modificado)
│   ├── localization_node.py       ← localización TB3 (Parte B, no modificado)
│   ├── particle_filter.py         ← filtro de partículas (Parte B)
│   ├── likelihood_field.py        ← campo de verosimilitud (Parte B)
│   └── map_publisher.py           ← publicador del mapa (Parte B)
├── map.pgm                        ← mapa real del laberinto (generado en Parte A)
└── map.yaml                       ← metadata del mapa
```

---

## Requisitos

- ROS 2 Humble
- Python 3 con: `numpy`, `scipy`, `opencv-python`, `cv_bridge`, `Pillow`
- Los dos paquetes buildeados: `despliegue_pkg` y `cono_detector_pkg`

---

## Build

```bash
cd ~/turtlebot3_ws
colcon build --packages-select despliegue_pkg --symlink-install
colcon build --paths src/despliegue_pkg/cono_detector_pkg --symlink-install
source install/setup.bash
```

`cono_detector_pkg` está anidado dentro de la carpeta de `despliegue_pkg`, y colcon
no desciende a buscar paquetes dentro de la carpeta de otro paquete — por eso
`--packages-select despliegue_pkg cono_detector_pkg` sólo builds `despliegue_pkg`
(ignora `cono_detector_pkg` con un warning silencioso). Hay que buildearlo aparte
con `--paths` apuntando directo a su carpeta.

---

## Cómo correr (4 terminales)

### Antes de empezar

En **todas** las terminales, siempre sourcer primero:
```bash
source ~/turtlebot3_ws/install/setup.bash
```

---

### Terminal 1 — Sistema de navegación (arrancar primero)

```bash
cd ~/turtlebot3_ws
source install/setup.bash
ros2 launch despliegue_pkg robot_real.launch.py
```

Esperá a ver estos mensajes antes de continuar:
```
[map_publisher] Map publisher listo
[localization_node] Mapa recibido
[localization_node] Likelihood recibido
```

---

### Terminal 2 — Detector de conos

```bash
cd ~/turtlebot3_ws
source install/setup.bash
ros2 run cono_detector_pkg detector_cono
```

---

### Terminal 3 — Robot real o bag de datos

**Con el robot real:**
```bash
# El robot publica sus topics automáticamente cuando está encendido y conectado
# No hace falta correr nada extra acá
```

**Con el bag (para validación previa):**
```bash
ros2 bag play /ruta/al/bag/laberinto_conos
```

---

### Terminal 4 — Monitoreo (opcional)

Para ver las detecciones del cono en tiempo real:
```bash
ros2 topic echo /tb4_0/cono_detectado
```

---

## Configuración en RViz

RViz se abre automáticamente con el launch. Configurar lo siguiente:

### 1. Fixed Frame
- En el panel izquierdo → **Global Options** → **Fixed Frame**
- Escribir: `map`

### 2. Agregar visualización del camino planificado
- Click en **Add** (abajo a la izquierda)
- Seleccionar **By topic** → `/planned_path` → **Path**
- El camino aparece como una línea verde cuando el robot navega hacia un cono

### 3. Agregar imagen de debug del detector (opcional)
- Abrir `rqt` en otra terminal: `rqt`
- **Plugins → Visualization → Image View**
- Seleccionar `/tb4_0/cono_detectado/debug_image`
- Muestra el contorno verde y el centroide azul del cono detectado

---

## 2D Pose Estimate — Paso crítico

**Esto hay que hacerlo cada vez que se inicia el sistema.**

El robot no sabe dónde está en el mapa hasta que vos se lo decís.

1. Ubicar visualmente en el mapa de RViz la zona donde está el robot físicamente
2. Click en **"2D Pose Estimate"** en la barra superior de RViz
3. **Click y arrastrar** sobre el mapa:
   - El punto donde clickeás = posición del robot
   - La dirección en que arrastrás = orientación (hacia dónde mira el robot)
4. Soltar — aparece una nube de flechitas rojas (partículas del filtro)
5. Esperar a que las flechitas converjan en un punto (unos segundos con el LIDAR activo)
6. Cuando convergen, el log muestra: `Localización convergió`
7. El robot pasa automáticamente a `EXPLORING` y empieza a moverse

**Si el filtro no converge:** el 2D Pose Estimate está en el lugar o ángulo equivocado. Repetir el paso apuntando a otra zona del mapa o con diferente orientación.

---

## Parámetros ajustables

Todos en `despliegue_pkg/robot_parte_c.py`, en el `__init__` de `RobotNavigatorC`:

| Parámetro | Valor actual | Descripción |
|---|---|---|
| `CONE_GOAL_OFFSET` | `0.40` m | Distancia a la que para antes del cono. Bajar para llegar más cerca (mínimo ~0.25 para no chocar con el mapa inflado) |
| `EXPLORE_LINEAR_SPEED` | `0.12` m/s | Velocidad de exploración |
| `EXPLORE_ANGULAR_SPEED` | `0.5` rad/s | Velocidad de giro al esquivar obstáculos |
| `EXPLORE_OBSTACLE_DIST` | `0.4` m | Distancia a la que empieza a girar |
| `INFLATION_RADIUS_CELLS` | `3` celdas (0.15 m) | Margen de seguridad alrededor de paredes (en `robot.py`) |

Para cambiar cualquier parámetro: editar el archivo con `nano` o cualquier editor y relanzar el sistema (no hace falta rebuild si el paquete está instalado con `--symlink-install`).

---

## Topics relevantes

| Topic | Tipo | Descripción |
|---|---|---|
| `/tb4_0/scan` | `LaserScan` | LIDAR del robot |
| `/tb4_0/odom` | `Odometry` | Odometría |
| `/tb4_0/oakd/rgb/preview/image_raw` | `Image` | Cámara RGB |
| `/tb4_0/cmd_vel` | `Twist` | Comandos de velocidad al robot |
| `/tb4_0/cono_detectado` | `PointStamped` | Posición del cono en frame del robot |
| `/tb4_0/cono_detectado/debug_image` | `Image` | Imagen con detección visualizada |
| `/map` | `OccupancyGrid` | Mapa de ocupación (Parte A) |
| `/belief` | `PoseArray` | Nube de partículas del filtro |
| `/estimated_pose` | `PoseStamped` | Pose estimada del robot |
| `/planned_path` | `Path` | Camino planificado por Theta* |

---

## Notas para el robot real (brecha sim-to-real)

- **Umbrales HSV del detector**: calibrados con el bag `laberinto_conos`. Bajo distintas condiciones de iluminación del lab pueden necesitar ajuste. Los umbrales están en `detector_cono_node.py` (`HSV_ROJO_BAJO_1/ALTO_1` y `HSV_ROJO_BAJO_2/ALTO_2`).
- **QoS BEST_EFFORT**: todos los topics del TB4 usan `BEST_EFFORT`. Si se agrega alguna suscripción nueva, verificar que use el QoS correcto o los mensajes no llegarán sin ningún error visible.
- **Offset del LIDAR**: el rplidar del TB4 está rotado 90° respecto de `base_link`. El offset está en `OFFSET_LIDAR_TB4 = 90.0` en `detector_cono_node.py` y se aplica en el cálculo de bearing.
- **Localización inicial**: el filtro de partículas necesita un 2D Pose Estimate razonablemente preciso (±0.5 m y ±30°) para converger. Si el robot se pierde durante la navegación, detener y repetir el 2D Pose Estimate.

---

## Origen del mapa

El mapa (`map.pgm` / `map.yaml`) fue generado en **Parte A** mediante:
1. Detección de marcadores ArUco con `aruco_detector_node`
2. Odometría con modelo de deltas `(δrot1, δtrans, δrot2)` con `odom_delta_node`
3. Optimización del grafo con **Graph SLAM** (cierre de lazo por re-observación de ArUcos)
4. Segunda pasada: proyección del LIDAR sobre la trayectoria corregida → grilla de ocupación log-odds

Es el mapa de la **corrida larga (46,8 min)**, el mismo que usa `navegacion_pkg` y
`entrega_parte_A/` (estandarizado el 03/07). El `map.pgm`/`map.yaml` que traía este
paquete al rescatarlo de `PRA_TpFinal` (02/07) era de una corrida anterior y quedó
desactualizado; se reemplazó el 03/07 por el de la entrega final.

Origen del mapa en coordenadas del mundo: `(-14.447, -21.394)` m.
