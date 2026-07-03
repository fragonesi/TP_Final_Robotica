# TP robotica - Parte A (Opción 3)

Dejo todo lo que fui haciendo hasta ahora, por qué lo hice así, qué bardos me aparecieron, como se solucionaron y que debería seguir. 

## El detector de ArUco

Armé `aruco_detector_node.py`. Se suscribe a la cámara, detecta los marcadores, calcula la pose 3D con solvePnP, y guarda todo en un CSV (id, distancia, posición, etc).

**El bardo más grande que tuve:** durante un buen rato el detector encontraba "candidatos" (veía algo cuadrado) en TODOS los frames, pero nunca decodificaba ningún ID. Probé de todo: subir resolución, sacar el CLAHE, jugar con los parámetros del detector... nada andaba. Pensé que era el blur o que la imagen era muy chica.

Al final la posta era mucho más boluda: **el diccionario estaba mal**. El código tenía `DICT_5X5_250` puesto por default, pero el marcador real es `DICT_4X4_50`. Armé un script (`aruco_dict_probe.py`) que prueba como 20 diccionarios distintos en paralelo contra la imagen y te dice cuál matchea. Apenas lo corrí, encontró el correcto al toque. Cambié el diccionario y pasé de 1 detección en todo el bag a 849.


También: el único tópico de imagen que hay es uno de baja resolución (el "preview"), así que el nodo agranda la imagen 4x antes de procesarla (con interpolación Lanczos, que conserva mejor el detalle que un resize normal).

### Cómo correrlo
```bash
ros2 run aruco_pkg aruco_detector_node --ros-args \
    -p image_topic:=/tb4_0/oakd/rgb/preview/image_raw \
    -p camera_info_topic:=/tb4_0/oakd/rgb/preview/camera_info \
    -p marker_length:=0.15 \
    -p upscale_factor:=4.0 \
    -p use_clahe:=false \
    -p aruco_dictionary:=DICT_4X4_50
```
(lo de `marker_length` capaz hay que medirlo bien si tenemos el marcador físico a mano, puse 0.15m de estimación)

## El modelo de ruido del sensor

Esto es para el Graph SLAM: cuando detectás un marcador, esa detección no es perfecta, tiene ruido. Y ese ruido es más grande cuanto más lejos está el marcador (tiene sentido, ocupa menos píxeles). Necesitamos decirle al optimizador del grafo "confiá más en esta detección" o "confiá menos", y eso se hace con una matriz de covarianza que depende de la distancia.

Para sacar esa curva usé el CSV de detecciones del bag corto (`aruco_estimation`, donde alguien sostiene el marcador a distintas distancias). Acá también hubo un bardo: pensé que iba a poder agrupar las detecciones por "paradas" (tipo, el marcador estuvo quieto a 0.5m, después a 1m, etc), pero **no los tomaba como  paradas reales**, es lucio moviendo el marcador a mano de forma continua, las pausas son cortas. Tuve que usar ventanitas de tiempo cortas y sacar la tendencia (porque dentro de una ventana corta el movimiento es casi una recta), y medir el ruido como lo que sobra después de sacar esa tendencia.

**Ojo:** como no hubo paradas limpias, el modelo es una aproximación, no algo super preciso. Si se te ocurre algo mejor banco!! ajajja

### Archivos
- `fit_noise_model.py`: lo corrés una vez con el CSV de detecciones y te tira `noise_model.json`
  ```bash
  python3 fit_noise_model.py /ruta/a/aruco_detections.csv
  ```
- `aruco_noise_model.py`: este va adentro del paquete de ROS (`aruco_pkg/aruco_pkg/`), porque el nodo de Graph SLAM lo va a importar para pedir la covarianza:
  ```python
  from aruco_pkg.aruco_noise_model import NoiseModel
  noise_model = NoiseModel('/ruta/a/noise_model.json')
  cov = noise_model.covariance(distancia_detectada)
  ```

## La odometría

`odom_delta_node.py` se suscribe a `/tb4_0/odom` y calcula los deltas tipo Thrun: cuánto giró antes de avanzar, cuánto avanzó, cuánto giró después. Esto es lo que después va a armar las restricciones de movimiento entre poses consecutivas en el grafo.

**Otro bardo:** el nodo no recibía NINGÚN mensaje de odometría, sin ningún error que lo explicara claramente (solo un warning de "QoS incompatible"). Resulta que el tópico de odom del bag se publica con una política llamada `BEST_EFFORT` (común en sensores reales), pero la suscripción por default de ROS2 pide `RELIABLE`, y si no coinciden, ROS2 simplemente no te entrega nada. Lo arreglé poniendo `BEST_EFFORT` explícito en la suscripción.


Ya lo corrí contra el bag `laberinto` completo (23 minutos, ~28000 lecturas) y la trayectoria que salió tiene pinta de laberinto real, nada raro. Hay un script (`plot_odom.py`) que te grafica la trayectoria y los deltas para chequear de un vistazo que todo tenga sentido.

### Cómo correrlo
```bash
ros2 run aruco_pkg odom_delta_node --ros-args \
    -p odom_topic:=/tb4_0/odom \
    -p log_csv_path:=odom_deltas.csv
```

## Qué falta (actualizado — GraphSLAM ya está armado)

**Update:** lo hicimos a mano con mínimos cuadrados (Gauss-Newton), que era una de
las opciones de la lista. Terminó siendo lo más cómodo y no mete dependencias raras
(g2o/GTSAM) que la consigna penaliza. Está todo en el paquete `TP_Final_Robotica`,
con autotests que pasan. Lo que YA está (✅) y lo que falta (⬜), en orden:

1. ✅ Estructura del grafo (poses + landmarks) → `graph_slam.py`
2. ✅ Edges de odometría (con los deltas)
3. ✅ Edges de observación (rango-bearing, covarianza que crece con la distancia)
4. ✅ Loop closure — sale solo cuando volvés a ver un tag ya visto (el ID da la
   asociación de datos gratis). Verificado en sintético: la deriva pasa de ~0.45 m a ~0.04 m.
5. ✅ Optimizar el grafo (Gauss-Newton disperso + keyframes; escala a las 28k lecturas
   del laberinto, solve en <0.1 s)
6. 🟡 Segunda pasada LIDAR → grilla de ocupación: el módulo está (`occupancy_grid.py`,
   exporta `.pgm`/`.yaml`), falta enchufarle los `tb4_0/scan` reales (¡ojo el QoS de nuevo,
   igual que con odom!)
7. ⬜ Exportar el mapa final del laberinto + landmarks por ID (el pipeline ya tira
   `landmarks.json` y la trayectoria) y mezclar con la parte B.

**El bloqueante de ahora:** correr todo sobre el bag del laberinto. Las detecciones ArUco
que tenemos son del bag corto, no del laberinto, así que falta bajar el bag (~8 GB), correrle
el detector, y pasarle odom+aruco al pipeline. Recién ahí hay loop closure de verdad.

**Cómo correrlo (cuando esté el bag):**
```bash
# 1) detector sobre el bag → detecciones del laberinto (en otra terminal: ros2 bag play <carpeta>)
ros2 run aruco_pkg aruco_detector_node --ros-args -p aruco_dictionary:=DICT_4X4_50 \
    -p image_topic:=/tb4_0/oakd/rgb/preview/image_raw -p log_csv_path:=laberinto_detections.csv
# 2) GraphSLAM end-to-end → mapa corregido + landmarks.json
python3 slam_pipeline.py --odom odom_deltas.csv --aruco laberinto_detections.csv --out-dir salida
# 3) verlo en RViz
ros2 run TP_Final_Robotica graph_slam_node --ros-args -p odom_csv:=odom_deltas.csv -p aruco_csv:=laberinto_detections.csv
```

## Dónde va cada archivo

Ahora es un workspace con los dos paquetes bajo `src/`:

```
src/
├── aruco_pkg/                      (percepción — lo de antes)
│   ├── aruco_pkg/
│   │   ├── aruco_detector_node.py
│   │   ├── odom_delta_node.py
│   │   ├── aruco_dict_probe.py     (solo para debuggear)
│   │   └── aruco_noise_model.py
│   ├── scripts/  (fit_noise_model.py, plot_odom.py)
│   └── config/   (calibracion_ejemplo.yaml)
└── TP_Final_Robotica/              (SLAM)
    └── TP_Final_Robotica/
        ├── graph_slam.py           núcleo GraphSLAM (Levenberg-Marquardt disperso + gating)
        ├── slam_pipeline.py        corré-todo: CSV → mapa + landmarks.json
        ├── graph_slam_node.py      nodo ROS para RViz (/belief, /landmarks, /poses_guardadas)
        ├── occupancy_grid.py       2da pasada → mapa .pgm/.yaml
        └── gslam.py                borrador previo (referencia)
```

Por las dudas, acordate de que los nodos hay que agregarlos en `setup.py` a los entry
points y recompilar con `colcon build` antes de que `ros2 run` los reconozca
(`aruco_detector_node` y `odom_delta_node` en `aruco_pkg`; `graph_slam_node` en
`TP_Final_Robotica`, ya agregado). Los `.csv`/bags no van a git (pesan demasiado);
el bag hay que bajarlo aparte y ponerlo en `src/aruco_pkg/`.

## Probado contra el bag REAL del laberinto (23 min) + alineación con la cátedra

Corrí toda la cadena contra el bag completo (detector → CSV → GraphSLAM → mapa).
El caso chico andaba, pero el bag entero destapó varios bardos que fui arreglando:

1. **OpenCV**: el detector usaba la API nueva (`ArucoDetector`); esta compu tiene
   OpenCV 4.5.4 (la de ROS Humble) con la API vieja → lo hice compatible con ambas.
   (Con eso detecta **~50 marcadores** del laberinto, 30/30 frames decodificados.)
2. **Divergencia**: el Gauss-Newton puro mandaba las poses a escala de km en el grafo
   grande → lo cambié por **Levenberg-Marquardt** (damping + aceptar solo pasos que
   bajan el error). No diverge más.
3. **~49k edges de observación** (uno por cada detección) saturaban el solver → ahora
   se **agregan por (keyframe, landmark)** promediando rango/bearing (~1.1k edges).
4. **Mapa borroso**: era la **extrínseca del LIDAR**. El TurtleBot4 tiene el láser
   girado **+90° (π/2)** respecto de base_link — lo confirmé en la **`parte0` de la
   cátedra** (`angle += math.pi/2`) y lo saqué del `tf_static` del bag (+ ~4 cm atrás).
   Sin eso las paredes rotan al girar y se emborronan.
5. **Pesos odom-dominantes**: la odometría del TB4 es buena, conviene confiar más en
   ella que en cada ArUco individual.
6. **Gating de outliers** (estilo `tp4`, Mahalanobis χ²): descarta detecciones ArUco
   espurias/mal asociadas y re-optimiza → **chi² 1403 → 537**.

**Evolución del chi²**: divergía → 11.806 (GN+agregación) → 1.403 (extrínseca+pesos)
→ **537 (gating)**. Resultado: converge, **50 landmarks** coherentes, y el mapa de
ocupación ya muestra **paredes reales** (se fue el borrón). Sigue **compacto** (~6×4 m,
que es el área realmente explorada).

**Extrínseca de la cámara**: aplicada (del `tf_static`). La rotación óptico→base ya
estaba bien (`x_base=z_opt`, `y_base=-x_opt`, idéntica a la matriz del tf); faltaba la
**traslación** — la oakd está ~6 cm detrás del origen del robot (t=(-0.0596,0,0.24)).
Es una corrección chica (la del LIDAR de 90° era la grande), el mapa cambia poco pero
la geometría queda correcta.

**Afinado del mapa (29/06)** — varios de los pendientes ya están hechos:

- **Cap de rango (~5 m)**: el bardo más visible del mapa eran unos **chorros largos
  espurios** que salían disparados (rayos de hasta ~8 m, el top 1%, que "liberaban"
  corredores falsos y clavaban impactos lejos). El laberinto entra de sobra en ~5 m
  (el 99% de los returns son <3.2 m, la trayectoria mide ~5.5×4.9 m), así que tratar
  como "sin retorno" todo lo que pase de 5 m los mata sin perder ninguna pared real.
  Es el cambio que más limpió el mapa.
- **Más cobertura de barridos** (800 → 4000): mejor consenso de paredes, menos borrón.
- **Filtrado por intensidad**: `scan_logger_node` ahora guarda las intensidades
  (`i0..iN`) y `occupancy_grid` descarta los haces con `intensity<=umbral` (lo que hace
  la `parte0`). **Ojo / hallazgo:** en *este* rplidar la intensidad es binaria (0 ó 47)
  y `intensity==0` coincide **exactamente** con los haces sin retorno (inf), que el
  código ya descartaba con `np.isfinite`. O sea, en estos datos el filtro no agrega nada
  nuevo; queda como red de seguridad y por alineación con la cátedra. Para re-capturar
  scans con intensidades sin reproducir el bag entero hay un extractor que deserializa
  `/tb4_0/scan` directo del `.db3`.

**Scan-matching contra el mapa (estilo Hector SLAM)** — atacamos el borrón residual:
el grosor de las paredes que quedaba **no era ruido sino error de pose** durante los
scans (jitter al interpolar la pose por timestamp + drift residual entre lazos). La
solución sin tocar el grafo: refinar la pose de **cada barrido** para que sus impactos
"calcen" sobre las paredes ya consensuadas.

- Cómo: en `occupancy_grid.py`, `LikelihoodField` arma un campo de verosimilitud
  (impactos acumulados + desenfoque Gaussiano, con muestreo bilineal y gradiente
  analítico) y `scan_match_pose` optimiza `(x,y,θ)` por **Gauss-Newton** sobre ese campo.
  Se hace en **pasadas coarse-to-fine** (blur decreciente). Pasos acotados por iteración
  y **rechazo** si la corrección es exagerada (>0.30 m / >12°) → un match dudoso no
  arrastra el barrido. Se activa con `scan_match=True` (flag `--sm-passes` del pipeline,
  default 2; el entregable se generó con 3).
- Resultado (medido sobre el `.pgm`): el chorro negro diagonal del centro **se adelgazó
  visiblemente**, ocupadas 3180 → ~2700 (paredes más finas, no más densas), y el ruido
  de sal-y-pimienta bajó. **Importante / honesto:** el grueso de la mejora viene con
  2 pasadas; de 3-4 en adelante son retoques marginales. Sigue **sin** ser un laberinto
  de líneas finas perfecto — un tramo con mucha rotación queda sucio —, pero es lo mejor
  alcanzable sin meterse con un pose-graph que use edges de LIDAR.

**Lo que todavía falta**: enchufar el `NoiseModel` ajustado en el grafo (hoy usa std por
defecto) y, si se quisiera el salto final de nitidez, edges de scan-matching **dentro**
del GraphSLAM (no solo en la 2da pasada).

**Rescate de obstáculos finos — la silla invisible (02/07)** — en el mapa de la
corrida nueva (`rosbag2_2026_07_01-12_45_54`, 46,8 min) faltaba la silla que está
junto al pouf. Diagnóstico con un mapa de *solo impactos* (sin la parte de liberar
celdas): las 4 patas están clarísimas en los datos (~1.000 retornos cada una), pero
una pata de ~2 cm no llena la celda de 5 cm — por cada rayo que impacta hay ~7 que
atraviesan la celda hacia paredes más lejanas y la "liberan". Con los pesos del
log-odds (`p_occ=0.7`/`p_free=0.4`, cociente 2:1) el consenso las borra. Se probó:
rebalancear pesos (hasta 0.8/0.48, cociente 17) → insuficiente; resolución 2,5 cm →
tampoco (el jitter de pose dispersa los impactos). La solución que funcionó:
**override por evidencia absoluta** — celdas con ≥N impactos crudos (`--min-hits-occ`,
escalar con `--max-scans`; usamos 500 con los 19.926 scans) y a >2 celdas de una
pared consensuada (para no engrosar el halo de las paredes) se exportan ocupadas.
Con scan-matching activo rescata solo 30 celdas: las 4 patas + tramos de paredes
finas que sufrían el mismo efecto. Ocupadas 2.131 → 2.227; las paredes no se
engrosan. También quedaron expuestos `--p-occ`/`--p-free` en el pipeline.
