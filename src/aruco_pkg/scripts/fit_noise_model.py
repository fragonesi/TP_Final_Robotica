import sys
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_and_filter(csv_path):
    df = pd.read_csv(csv_path)

    # Nos quedamos solo con el marker_id más frecuente: el resto son
    # falsos positivos esporádicos (ver ejemplo real: id=17 con 3 detecciones
    # a distancias inconsistentes entre sí, vs. id=10 con 800+ detecciones).
    main_id = df['marker_id'].value_counts().idxmax()
    n_total = len(df)
    df = df[df['marker_id'] == main_id].reset_index(drop=True)
    print(f"marker_id principal: {main_id} ({len(df)}/{n_total} detecciones retenidas)")

    return df.sort_values('timestamp').reset_index(drop=True)


def windowed_noise(df, window_n=20, step_n=10):
    """Estima el ruido local mediante ventanas deslizantes con detrending
    """
    rows = []
    n = len(df)
    t = df['timestamp'].values
    for start in range(0, n - window_n, step_n):
        end = start + window_n
        t_win = t[start:end]
        t_centered = t_win - t_win.mean()

        d = {}
        for axis in ['tx', 'ty', 'tz']:
            y = df[axis].values[start:end]
            # ajuste lineal y residuo (detrend)
            A = np.vstack([np.ones_like(t_centered), t_centered]).T
            coeffs, _, _, _ = np.linalg.lstsq(A, y, rcond=None)
            y_fit = A @ coeffs
            resid = y - y_fit
            d[f'{axis}_std'] = float(np.std(resid))

        rows.append({
            'dist_mean': float(df['distancia_m'].values[start:end].mean()),
            'n': window_n,
            **d,
        })

    return pd.DataFrame(rows)


def fit_linear(distances, stds):
    """Ajusta std = a + b*distancia por mínimos cuadrados. Devuelve (a, b)."""
    A = np.vstack([np.ones_like(distances), distances]).T
    coeffs, _, _, _ = np.linalg.lstsq(A, stds, rcond=None)
    a, b = coeffs
    # Piso mínimo de ruido: nunca devolver std=0 aunque el ajuste lo sugiera
    # a corta distancia (sería sobre-confiar en el sensor).
    a = max(a, 1e-4)
    return float(a), float(b)


def main():
    if len(sys.argv) < 2:
        print("Uso: python3 fit_noise_model.py /ruta/a/aruco_detections.csv")
        sys.exit(1)

    csv_path = sys.argv[1]
    df = load_and_filter(csv_path)
    windows = windowed_noise(df)

    if len(windows) < 2:
        print("Muy pocas ventanas generadas, revisar window_n/step_n o el CSV de entrada.")
        sys.exit(1)

    print(f"\n{len(windows)} ventanas generadas:")
    print(windows.describe()[['dist_mean', 'tx_std', 'ty_std', 'tz_std']].to_string())

    dist = windows['dist_mean'].values
    model = {}
    for axis in ['tx', 'ty', 'tz']:
        a, b = fit_linear(dist, windows[f'{axis}_std'].values)
        model[axis] = {'a': a, 'b': b}
        print(f"\n{axis}: std(d) = {a:.5f} + {b:.5f} * d")

    out_path = 'noise_model.json'
    with open(out_path, 'w') as f:
        json.dump(model, f, indent=2)
    print(f"\nModelo guardado en {out_path}")

    # Gráfico de diagnóstico: datos reales vs. recta ajustada por eje
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    d_line = np.linspace(dist.min(), dist.max(), 100)
    for ax, axis in zip(axes, ['tx', 'ty', 'tz']):
        ax.scatter(dist, windows[f'{axis}_std'], alpha=0.5, label='ventanas (detrended)')
        a, b = model[axis]['a'], model[axis]['b']
        ax.plot(d_line, a + b * d_line, 'r--', label='ajuste lineal')
        ax.set_xlabel('distancia (m)')
        ax.set_ylabel(f'std {axis} (m)')
        ax.set_title(axis)
        ax.legend()
    plt.tight_layout()
    plt.savefig('noise_model_fit.png', dpi=120)
    print("Gráfico guardado en noise_model_fit.png")


if __name__ == '__main__':
    main()