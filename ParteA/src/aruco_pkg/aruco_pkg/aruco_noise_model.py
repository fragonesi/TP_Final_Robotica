"""Modelo de ruido del sensor ArUco, consumido por el back-end de GraphSLAM.

Una detección ArUco no es perfecta: el error crece con la distancia (el marcador
ocupa menos píxeles, las esquinas se ubican peor). El grafo necesita saber cuánto
confiar en cada observación → este módulo entrega la covarianza (o su inversa, la
matriz de información) que pondera cada edge de observación.

Modelo: `std(d) = a + b·d` por eje, con (a, b) ajustados por `fit_noise_model.py`
sobre el bag corto `aruco_estimation` y guardados en `noise_model.json`. La
covarianza se asume diagonal en (tx, ty, tz). El piso `min_std` evita std=0
(sobre-confianza) a distancia corta.

Uso desde el back-end:
    from aruco_pkg.aruco_noise_model import NoiseModel
    nm = NoiseModel('noise_model.json')
    info = nm.information(distancia_detectada)   # matriz de información del edge
"""
import json
import numpy as np


class NoiseModel:
    def __init__(self, json_path, min_std=1e-4):
        with open(json_path, 'r') as f:
            self.params = json.load(f)
        self.min_std = min_std

    def _std(self, axis, distance):
        a = self.params[axis]['a']
        b = self.params[axis]['b']
        return max(a + b * distance, self.min_std)

    def std_vector(self, distance):
        """Devuelve (std_tx, std_ty, std_tz) para una distancia dada."""
        return (
            self._std('tx', distance),
            self._std('ty', distance),
            self._std('tz', distance),
        )

    def covariance(self, distance):
        """Matriz de covarianza 3x3 diagonal (tx, ty, tz) para esa distancia.

        Se asume diagonal (ejes independientes) como primera aproximación.
        Si en el informe técnico quieren justificar correlación entre ejes,
        se puede extender a partir de los residuos crudos del CSV.
        """
        std_tx, std_ty, std_tz = self.std_vector(distance)
        return np.diag([std_tx ** 2, std_ty ** 2, std_tz ** 2])

    def information(self, distance):
        """Matriz de información (inversa de la covarianza) para el edge."""
        cov = self.covariance(distance)
        return np.linalg.inv(cov)


if __name__ == '__main__':
    import sys
    if len(sys.argv) < 3:
        print("Uso: python3 aruco_noise_model.py noise_model.json distancia_m")
        sys.exit(1)
    nm = NoiseModel(sys.argv[1])
    d = float(sys.argv[2])
    print(f"A {d} m -> std (tx,ty,tz) = {nm.std_vector(d)}")
    print(f"Covarianza:\n{nm.covariance(d)}")
