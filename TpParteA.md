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

## Qué falta (segun claude y lo que entendi de la consigna xd)

Esto es lo que viene, en orden:

1. Armar la estructura del grafo (nodos = poses + landmarks). Todavía no decidimos si usamos g2o, GTSAM, o lo hacemos a mano con `scipy.optimize.least_squares`.
2. Edges de odometría (con los deltas que ya tenemos)
3. Edges de observación (con la covarianza que ya tenemos)
4. Loop closure — es obligatorio según la consigna
5. Optimizar el grafo
6. Segunda pasada con el LIDAR para armar la grilla de ocupación (ojo con el tema QoS de nuevo)
7. Exportar el mapa final + el archivo de landmarks por ID y mezclar con lo que están haciendo las chicas de la parte B

## Dónde va cada archivo

```
aruco_pkg/
├── aruco_pkg/
│   ├── aruco_detector_node.py
│   ├── odom_delta_node.py
│   ├── aruco_dict_probe.py     (este podés dejarlo suelto también, es solo para debuggear)
│   └── aruco_noise_model.py
├── scripts/
│   ├── fit_noise_model.py
│   └── plot_odom.py
└── config/
    └── noise_model.json
```

Por las dudas, acordate de que `aruco_detector_node` y `odom_delta_node` hay que agregarlos en `setup.py` a los entry points y despues recompilar con `colcon build --packages-select aruco_pkg` antes de que `ros2 run` los reconozca. El resto son todos scripts de python para ayudarnos a solucionar alguna que otra cosa o ver graficamente otras.
