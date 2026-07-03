# Entrega — Parte A (Opción 3: Features con Cámara)

Entregables finales del GraphSLAM sobre el bag del laberinto (~23 min), generados
con la cadena completa: detector ArUco → odometría → GraphSLAM (odom + observaciones
ArUco rango-bearing + cierre de lazo por ID) → grilla de ocupación (2da pasada LIDAR).

## Archivos

- **`mapa.pgm`** / **`mapa.yaml`** — grilla de ocupación en formato `map_server` de ROS
  (`0`=ocupado, `254`=libre, `205`=desconocido), lista para A*/Dijkstra en las Partes B/C.
  Resolución 0.05 m/celda.
- **`landmarks.json`** — hitos ArUco por ID en el marco del mapa: `{id: [x, y]}`.
- **`trayectoria_opt.csv`** — trayectoria corregida por keyframe (`x, y, theta`).
- **`mapa.png`** / **`trayectoria.png`** — vistas de diagnóstico.

## Cómo se regenera

Desde `src/TP_Final_Robotica/TP_Final_Robotica/`, con los CSV del laberinto
(odometría, detecciones ArUco y barridos LIDAR):

```bash
python3 slam_pipeline.py --odom odom_deltas.csv --aruco laberinto_detections.csv \
    --scans scans.csv --out-dir salida \
    --noise-model ../../aruco_pkg/aruco_pkg/noise_model.json
```

El `--noise-model` enchufa la covarianza de observación fiteada (`std = a + b·d`),
propagada de cartesiano a rango-bearing con término sistemático de escala del
`marker_length`: confía fuerte en el *bearing* (mejora el cierre de lazo) y deja el
*rango* apropiadamente flojo. Es lo que afina las paredes (deja de salir borroso).
