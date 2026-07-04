# Entrega — Parte A (Opción 3: Features con Cámara)

Entregables finales del GraphSLAM sobre el bag de la **corrida larga del laberinto**
(`rosbag2_2026_07_01-12_45_54`, 46,8 min, múltiples lazos), generados con la cadena
completa: detector ArUco → odometría → GraphSLAM (odom + observaciones ArUco
rango-bearing + cierre de lazo por ID) → grilla de ocupación (2da pasada LIDAR con
scan-matching y rescate de obstáculos finos). 46 landmarks; χ² 54.442 → 1.686.

## Archivos

- **`mapa.pgm`** / **`mapa.yaml`** — grilla de ocupación en formato `map_server` de ROS
  (`0`=ocupado, `254`=libre, `205`=desconocido), lista para A*/Dijkstra en las Partes B/C.
  Resolución 0.05 m/celda. **Regenerado 03/07** con el fix del clamp del log-odds
  (ver abajo) — las paredes internas del laberinto ahora son visibles.
- **`landmarks.json`** — hitos ArUco por ID en el marco del mapa: `{id: [x, y]}`.
  Sin cambios respecto de la versión anterior (el fix solo afecta la grilla).
- **`trayectoria_opt.csv`** — trayectoria corregida por keyframe (`x, y, theta`).
  Sin cambios.
- **`mapa.png`** / **`trayectoria.png`** — vistas de diagnóstico.

## Fix 03/07: paredes internas del laberinto invisibles en el mapa

Hasta esta regeneración, `mapa.pgm` mostraba bien el **perímetro** del laberinto
pero casi ninguna de sus **paredes internas** (las que separan pasillos), pese a que
`trayectoria.png` mostraba un recorrido con muchos giros/lazos que solo se explican
si esas paredes existen. La causa: `OccupancyGridMap` (en `occupancy_grid.py`)
aplicaba el clip del log-odds (`clamp=5.0`, heredado del libro de texto) en **cada**
actualización, no solo al resultado final. Con los `p_occ`/`p_free` por defecto
(0.7/0.4), eso satura una celda después de apenas ~6 impactos o ~12 pasadas-libres
consecutivas — insuficiente para un laberinto recorrido en varios lazos, donde una
misma celda de pared puede acumular miles de impactos a lo largo de la corrida. El
resultado termina dependiendo del **orden reciente** de los eventos en vez de la
mayoría histórica: celdas con miles de impactos LIDAR reales terminaban marcadas
libres solo porque las últimas ~12 pasadas antes de que el robot dejara de pasar por
ahí fueron rayos que la atravesaban sin impactar.

Se verificó contra los datos reales (`Rosbags/corrida2_run/`): el histograma crudo de
impactos por celda (`grid.hits`) traza clarísimamente todo el interior del laberinto,
mientras que el mapa de probabilidad (`grid.prob()`) lo perdía casi por completo —
por ejemplo, una celda con 7.784 impactos crudos quedaba con log-odds `-5.0`
(saturado "libre"). Subiendo `clamp` a **50** (10x, sigue siendo un valor moderado,
no un extremo arbitrario — probado también con 100 y 1000 sin mejoras adicionales)
esas mismas celdas recuperan correctamente `prob≈1.0`. El rescate manual por
impactos (`--min-hits-occ`) que antes rescataba 30 celdas ahora solo necesita
rescatar 8 — señal de que el consenso log-odds por sí solo ya resuelve casi todo.
`clamp` quedó expuesto como parámetro de `slam_pipeline.py` (`--clamp`, default 50).

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

(`--clamp 50` ya es el default, no hace falta pasarlo explícito.)

El `--noise-model` enchufa la covarianza de observación fiteada (`std = a + b·d`),
propagada de cartesiano a rango-bearing: confía fuerte en el *bearing* (mejora el
cierre de lazo) y deja el *rango* apropiadamente flojo. `--sm-passes` refina la
trayectoria con scan-matching coarse-to-fine antes de rasterizar, y `--min-hits-occ`
rescata obstáculos finos (patas de silla) que el consenso del log-odds borraría.
Detalles y justificación en `TpParteA.md` (entradas 02/07 y 03/07) y el README raíz.
