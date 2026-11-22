#!/usr/bin/env python3
"""Make comparable top views from saved PCDs and TUM trajectories."""
from pathlib import Path
import argparse
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--session', type=Path, required=True)
parser.add_argument('--repeat', type=int, default=1)
args = parser.parse_args()
SESSION = args.session
OUT = SESSION / 'figures'
OUT.mkdir(exist_ok=True)
MODES = ('baseline', 'delay', 'loss')
LABELS = {'baseline': 'Без помех', 'delay': 'Задержка 100 мс', 'loss': 'Потери 1%'}
COLORS = ('#dc2626', '#2563eb', '#16a34a')


def read_sample(path, stride=30):
    # Uniform deterministic sampling in file order; no synthetic alignment.
    points = []
    with path.open() as f:
        for line in f:
            if line.startswith('DATA '):
                if line.strip() != 'DATA ascii':
                    raise ValueError(f'Expected ASCII PCD: {path}')
                break
        for i, line in enumerate(f):
            if i % stride:
                continue
            xyz = np.fromstring(line, sep=' ', count=3)
            if xyz.size == 3 and np.isfinite(xyz).all() and np.linalg.norm(xyz) > 0.01:
                points.append(xyz)
    return np.asarray(points)


maps = {}
trajectories = {}
for mode in MODES:
    maps[mode] = []
    trajectories[mode] = []
    for i in range(3):
        folder = SESSION / mode / f'run_{args.repeat}/log'
        maps[mode].append(read_sample(folder / f'robot_{i}_pcd_wrt_global.pcd'))
        trajectories[mode].append(np.loadtxt(folder / f'robot_{i}.tum')[:, 1:4])

all_xy = np.concatenate([p[:, :2] for mode in MODES for p in maps[mode]])
lower, upper = np.quantile(all_xy, [0.001, 0.999], axis=0)
margin = (upper - lower) * 0.04
lower, upper = lower - margin, upper + margin

# Find a densely populated region where all three baseline maps overlap. The
# same chosen coordinates are used for all regimes, so panels are comparable.
bins_x = np.arange(np.floor(lower[0] / 4) * 4, np.ceil(upper[0] / 4) * 4 + 4, 4)
bins_y = np.arange(np.floor(lower[1] / 4) * 4, np.ceil(upper[1] / 4) * 4 + 4, 4)
density = []
for points in maps['baseline']:
    h, _, _ = np.histogram2d(points[:, 0], points[:, 1], bins=(bins_x, bins_y))
    density.append(gaussian_filter(h, 1.2))
score = np.minimum.reduce(density)
if np.max(score) <= 0:
    raise ValueError('No three-robot overlapping region found')
ix, iy = np.unravel_index(np.argmax(score), score.shape)
center = np.array([(bins_x[ix] + bins_x[ix + 1]) / 2, (bins_y[iy] + bins_y[iy + 1]) / 2])
zoom_lo, zoom_hi = center - 20, center + 20


def draw(mode, zoom):
    fig, ax = plt.subplots(figsize=(9, 7), constrained_layout=True)
    for i, points in enumerate(maps[mode]):
        ax.scatter(points[:, 0], points[:, 1], c=COLORS[i], s=0.22, alpha=.17, rasterized=True, label=f'Карта робота {i}')
    for i, trajectory in enumerate(trajectories[mode]):
        ax.plot(trajectory[:, 0], trajectory[:, 1], color=COLORS[i], linewidth=1.8, label=f'Траектория {i}')
        ax.scatter(*trajectory[0, :2], color=COLORS[i], edgecolor='black', s=28, zorder=5)
    ax.set_xlim(*( (zoom_lo[0], zoom_hi[0]) if zoom else (lower[0], upper[0]) ))
    ax.set_ylim(*( (zoom_lo[1], zoom_hi[1]) if zoom else (lower[1], upper[1]) ))
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('x, м')
    ax.set_ylabel('y, м')
    ax.set_title(f"{LABELS[mode]} — {'участок перекрытия' if zoom else 'общая карта и траектории'}")
    ax.grid(alpha=.18)
    ax.legend(loc='upper left', fontsize=7, framealpha=.92, markerscale=8, ncol=2)
    fig.savefig(OUT / f'{mode}_{"zoom" if zoom else "overview"}.png', dpi=160)
    plt.close(fig)


for mode in MODES:
    draw(mode, False)
    draw(mode, True)
(OUT / 'figure_metadata.json').write_text(json.dumps({'source': 'PCD and TUM in each run; no extra alignment', 'sample_stride': 30, 'map_bounds_xy': [lower.tolist(), upper.tolist()], 'zoom_center_xy': center.tolist(), 'zoom_size_m': 40, 'selection': 'densest common area of three baseline maps after 4 m binning and Gaussian smoothing'}, indent=2))
print('overlap center:', center, 'bounds:', lower, upper)
