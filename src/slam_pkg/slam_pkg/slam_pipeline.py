"""Pipeline de GraphSLAM: corre la optimización desde los CSV y exporta resultados.

Es el "corré-todo" de la Parte A: toma la odometría (y, si está, las detecciones
ArUco), optimiza el grafo con graph_slam.py y deja listos los entregables.

Uso:
    python3 slam_pipeline.py --odom odom_deltas.csv \
        [--aruco aruco_detections.csv] [--out-dir salida]

Genera en out-dir:
    - trayectoria.png       odometría cruda vs trayectoria optimizada + landmarks
    - landmarks.json        ENTREGABLE: {marker_id: [x, y]} (hitos por ID)
    - trayectoria_opt.csv   x, y, theta por keyframe (trayectoria corregida)
"""
import argparse
import json
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from graph_slam import build_from_csv


def run(odom_csv, aruco_csv=None, out_dir='slam_out', scans_csv=None,
        iterations=30, gate_chi2=13.8, max_scans=4000, range_cap=5.0,
        intensity_min=0.0, scan_match=True, sm_passes=2,
        p_occ=0.7, p_free=0.4, min_hits_occ=None, **kw):
    os.makedirs(out_dir, exist_ok=True)

    g = build_from_csv(odom_csv, aruco_csv, **kw)
    raw = np.array([p.copy() for p in g.poses])   # odometría cruda (estim. inicial)
    print(f"poses(keyframes)={g.n_poses}  landmarks={len(g.landmark_ids)}  "
          f"edges_odom={len(g.odom_edges)}  edges_obs={len(g.obs_edges)}")

    g.optimize(iterations=iterations)
    # Gating de outliers (estilo tp4) + re-optimización, hasta que no quede ninguno.
    if gate_chi2 and gate_chi2 > 0:
        for _ in range(3):
            n = g.gate_observations(gate_chi2)
            if n == 0:
                break
            print(f"  gating χ²>{gate_chi2}: descartadas {n} obs outlier "
                  f"(quedan {len(g.obs_edges)}, {len(g.landmark_ids)} landmarks); re-optimizando...")
            g.optimize(iterations=iterations)
    opt = np.array(g.poses)

    # --- gráfico de diagnóstico ---
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.plot(raw[:, 0], raw[:, 1], '-', color='gray', lw=1.0, label='odometría cruda')
    ax.plot(opt[:, 0], opt[:, 1], '-', color='C0', lw=1.6, label='GraphSLAM (corregida)')
    ax.scatter(opt[0, 0], opt[0, 1], c='green', zorder=5, label='inicio')
    ax.scatter(opt[-1, 0], opt[-1, 1], c='red', zorder=5, label='fin')
    for lid in g.landmark_ids:
        lx, ly = g.landmarks[lid]
        ax.scatter(lx, ly, marker='*', s=180, c='orange', edgecolor='k', zorder=6)
        ax.annotate(f'id {lid}', (lx, ly), textcoords='offset points', xytext=(6, 6))
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title('Trayectoria: odometría vs GraphSLAM')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    png = os.path.join(out_dir, 'trayectoria.png')
    fig.savefig(png, dpi=130, bbox_inches='tight')
    print('plot       →', png)

    # --- entregables ---
    landmarks = {int(lid): [float(g.landmarks[lid][0]), float(g.landmarks[lid][1])]
                 for lid in g.landmark_ids}
    lm_path = os.path.join(out_dir, 'landmarks.json')
    with open(lm_path, 'w') as f:
        json.dump(landmarks, f, indent=2)
    traj_path = os.path.join(out_dir, 'trayectoria_opt.csv')
    np.savetxt(traj_path, opt, delimiter=',', header='x,y,theta', comments='')
    print('landmarks  →', lm_path)
    print('trayectoria→', traj_path)

    # --- 2da pasada opcional: grilla de ocupación con LIDAR ---
    if scans_csv:
        from occupancy_grid import build_grid_from_scans
        grid = build_grid_from_scans(opt, g.pose_times, scans_csv, max_scans=max_scans,
                                     range_cap=range_cap, intensity_min=intensity_min,
                                     scan_match=scan_match, sm_passes=sm_passes,
                                     p_occ=p_occ, p_free=p_free)
        prefix = os.path.join(out_dir, 'mapa')
        grid.export_ros_map(prefix, min_hits_occ=min_hits_occ)
        if min_hits_occ is not None:
            extra = int((grid._occ_mask(0.65, min_hits_occ) & ~(grid.prob() >= 0.65)).sum())
            print(f"  override por impactos (hits>={min_hits_occ}, aislado de paredes): "
                  f"{extra} celdas rescatadas del consenso")
        # el PNG muestra la MISMA ocupación exportada (incluye el override de impactos)
        occ = grid.to_occupancy(min_hits_occ=min_hits_occ)
        img = np.full(occ.shape, 0.5)
        img[occ == 0] = 0.0
        img[occ == 100] = 1.0
        figm, axm = plt.subplots(figsize=(8, 8))
        axm.imshow(img, origin='lower', cmap='gray_r', vmin=0.0, vmax=1.0)
        axm.set_title('Grilla de ocupación (LIDAR + trayectoria corregida)')
        figm.savefig(os.path.join(out_dir, 'mapa.png'), dpi=130, bbox_inches='tight')
        print('mapa       →', prefix + '.pgm/.yaml + mapa.png')

    return g


def main():
    ap = argparse.ArgumentParser(description='Pipeline GraphSLAM desde CSV')
    ap.add_argument('--odom', required=True, help='CSV de odom_delta_node')
    ap.add_argument('--aruco', default=None, help='CSV de aruco_detector_node (opcional)')
    ap.add_argument('--out-dir', default='slam_out')
    ap.add_argument('--scans', default=None, help='CSV de scan_logger_node (para la grilla)')
    ap.add_argument('--iters', type=int, default=80, help='iteraciones de Levenberg-Marquardt')
    ap.add_argument('--kf-trans', type=float, default=0.10, help='umbral keyframe [m]')
    ap.add_argument('--kf-rot-deg', type=float, default=10.0, help='umbral keyframe [grados]')
    ap.add_argument('--gate-chi2', type=float, default=13.8,
                    help='umbral χ²(2gl) para gating de outliers ArUco (0 = desactivar)')
    ap.add_argument('--max-scans', type=int, default=4000,
                    help='barridos LIDAR a usar en la grilla (más = paredes más nítidas)')
    ap.add_argument('--range-cap', type=float, default=5.0,
                    help='descartar como impacto los rangos > este umbral [m] (0 = sin cap)')
    ap.add_argument('--intensity-min', type=float, default=0.0,
                    help='descartar haces con intensity <= umbral (si el CSV trae intensidades)')
    ap.add_argument('--no-scan-match', action='store_true',
                    help='desactivar el scan-matching de la grilla (refinamiento de pose por barrido)')
    ap.add_argument('--sm-passes', type=int, default=2,
                    help='pasadas de scan-matching coarse-to-fine (más = paredes más finas, más costo)')
    ap.add_argument('--p-occ', type=float, default=0.7,
                    help='prob. del modelo inverso para la celda del impacto (subir para que '
                         'sobrevivan obstáculos finos tipo patas de silla)')
    ap.add_argument('--p-free', type=float, default=0.4,
                    help='prob. del modelo inverso para las celdas atravesadas (acercar a 0.5 '
                         'para que las pasadas de rayo no borren obstáculos finos)')
    ap.add_argument('--min-hits-occ', type=int, default=None,
                    help='celdas con >= este nro de impactos LIDAR se exportan ocupadas aunque '
                         'el consenso log-odds las marque libres (rescata obstáculos finos que '
                         'no llenan la celda, p.ej. patas de silla). Escalar con --max-scans.')
    ap.add_argument('--noise-model', default=None,
                    help='noise_model.json (fit_noise_model.py): covarianza ArUco fiteada '
                         'en vez de los std por defecto. Mejora mucho la nitidez del mapa.')
    ap.add_argument('--scale-uncertainty', type=float, default=0.08,
                    help='incertidumbre relativa del marker_length: término sistemático (k·r) '
                         'sumado al rango ArUco (solo con --noise-model)')
    a = ap.parse_args()
    run(a.odom, a.aruco, a.out_dir, scans_csv=a.scans, iterations=a.iters,
        gate_chi2=a.gate_chi2, max_scans=a.max_scans,
        range_cap=(None if a.range_cap <= 0 else a.range_cap),
        intensity_min=a.intensity_min, scan_match=(not a.no_scan_match), sm_passes=a.sm_passes,
        p_occ=a.p_occ, p_free=a.p_free, min_hits_occ=a.min_hits_occ,
        kf_trans=a.kf_trans, kf_rot=np.deg2rad(a.kf_rot_deg),
        noise_model_path=a.noise_model, scale_uncertainty=a.scale_uncertainty)


if __name__ == '__main__':
    main()
