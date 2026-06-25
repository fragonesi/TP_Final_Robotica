import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    if len(sys.argv) < 2:
        sys.exit(1)

    df = pd.read_csv(sys.argv[1])
    df = df.sort_values('timestamp').reset_index(drop=True)
    df['t_rel'] = df['timestamp'] - df['timestamp'].iloc[0]

    print(f"Total de filas: {len(df)}")
    print(f"Duración total: {df['t_rel'].iloc[-1]:.1f} s")
    print(f"delta_trans total acumulado: {df['delta_trans'].sum():.3f} m")
    print(f"delta_rot2 total acumulado: {np.degrees(df['delta_rot2'].sum()):.1f} grados")
    print()
    print("Resumen delta_trans (m por paso):")
    print(df['delta_trans'].describe())
    print()
    print("Resumen dt (s entre lecturas):")
    print(df['dt'].describe())

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    # x,y crudo reportado por el tópico de odometría (no reconstruido a partir
    # de los deltas; esto es la trayectoria "tal cual" la publica el robot)
    axes[0, 0].plot(df['x'], df['y'], '-', linewidth=0.8)
    axes[0, 0].scatter(df['x'].iloc[0], df['y'].iloc[0], color='green', label='inicio', zorder=5)
    axes[0, 0].scatter(df['x'].iloc[-1], df['y'].iloc[-1], color='red', label='fin', zorder=5)
    axes[0, 0].set_xlabel('x (m)')
    axes[0, 0].set_ylabel('y (m)')
    axes[0, 0].set_title('Trayectoria (x,y) reportada por /tb4_0/odom')
    axes[0, 0].axis('equal')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # theta en el tiempo (en grados, más intuitivo para inspección visual)
    axes[0, 1].plot(df['t_rel'], np.degrees(df['theta']))
    axes[0, 1].set_xlabel('tiempo (s)')
    axes[0, 1].set_ylabel('theta (grados)')
    axes[0, 1].set_title('Orientación en el tiempo')
    axes[0, 1].grid(True, alpha=0.3)

    # delta_trans por paso en el tiempo -> permite ver de un vistazo cuándo
    # el robot avanza de verdad vs. cuándo está quieto/girando in-place
    axes[1, 0].plot(df['t_rel'], df['delta_trans'])
    axes[1, 0].set_xlabel('tiempo (s)')
    axes[1, 0].set_ylabel('delta_trans (m)')
    axes[1, 0].set_title('Desplazamiento por paso')
    axes[1, 0].grid(True, alpha=0.3)

    # delta_rot2 por paso (en grados) -> picos = giros rápidos
    axes[1, 1].plot(df['t_rel'], np.degrees(df['delta_rot2']))
    axes[1, 1].axhline(0, color='gray', linewidth=0.5)
    axes[1, 1].set_xlabel('tiempo (s)')
    axes[1, 1].set_ylabel('delta_rot2 (grados)')
    axes[1, 1].set_title('Rotación por paso')
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('odom_trajectory.png', dpi=120)
    print("\nGráfico guardado en odom_trajectory.png")


if __name__ == '__main__':
    main()