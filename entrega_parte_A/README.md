# Entrega — Parte A (Opción 3: Features con Cámara)

Entregables finales del GraphSLAM sobre el bag de la **corrida larga del laberinto**
(`rosbag2_2026_07_01-12_45_54`, 46,8 min, múltiples lazos), generados con la cadena
completa: detector ArUco → odometría → GraphSLAM (odom + observaciones ArUco
rango-bearing + cierre de lazo por ID) → grilla de ocupación (2da pasada LIDAR con
scan-matching y rescate de obstáculos finos). 46 landmarks; χ² 54.442 → 6.281.

## Archivos

- **`mapa.pgm`** / **`mapa.yaml`** — grilla de ocupación en formato `map_server` de ROS
  (`0`=ocupado, `254`=libre, `205`=desconocido), lista para A*/Dijkstra en las Partes B/C.
  Resolución 0.05 m/celda.
- **`landmarks.json`** — hitos ArUco por ID en el marco del mapa: `{id: [x, y]}`.
- **`trayectoria_opt.csv`** — trayectoria corregida por keyframe (`x, y, theta`).
- **`mapa.png`** / **`trayectoria.png`** — vistas de diagnóstico.

## Cómo se regenera

Desde `src/slam_pkg/slam_pkg/`, con los CSV de la corrida (odometría, detecciones
ArUco y barridos LIDAR):

```bash
python3 slam_pipeline.py \
    --odom  odom_deltas.csv \
    --aruco laberinto_detections.csv \
    --scans scans.csv \
    --noise-model ../../aruco_pkg/aruco_pkg/noise_model.json \
    --sm-passes 3 --max-scans 19926 --min-hits-occ 500 \
    --out-dir salida
```

El `--noise-model` enchufa la covarianza de observación fiteada (`std = a + b·d`),
propagada de cartesiano a rango-bearing: confía fuerte en el *bearing* (mejora el
cierre de lazo) y deja el *rango* apropiadamente flojo. `--sm-passes` refina la
trayectoria con scan-matching coarse-to-fine antes de rasterizar, y `--min-hits-occ`
rescata obstáculos finos (patas de silla) que el consenso del log-odds borraría.
Detalles y justificación en `TpParteA.md` (entrada 02/07) y el README raíz.
