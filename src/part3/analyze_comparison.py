#!/usr/bin/env python3
"""Compare raw input, isolated and collaborative runs with fixed initial registration."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
GT_DIR = ROOT / 'datasets/S3E/ground_truth_campus_road_1'
INPUT_DIR = ROOT / 'results/part3/input_odom'
T0 = 1660988157.6098995
CALIBRATION_S = 100.
MAX_REFERENCE_DT = .55
MAX_TRAJECTORY_DT = .15
ASSIGNMENT = ('alpha', 'bob', 'carol')  # Inferred from the earlier joint fit; verify provenance.


def nearest(query, reference, tolerance):
    j = np.searchsorted(reference[:, 0], query)
    lo = np.clip(j - 1, 0, len(reference) - 1)
    hi = np.clip(j, 0, len(reference) - 1)
    idx = np.where(abs(reference[hi, 0] - query) < abs(reference[lo, 0] - query), hi, lo)
    return idx, abs(reference[idx, 0] - query) <= tolerance


def transform_fit(x, y, planar=False):
    if planar:
        xmean, ymean = x.mean(0), y.mean(0)
        u, _, vt = np.linalg.svd((x[:, :2] - xmean[:2]).T @ (y[:, :2] - ymean[:2]))
        r2 = vt.T @ np.diag([1., np.linalg.det(vt.T @ u.T)]) @ u.T
        r = np.eye(3)
        r[:2, :2] = r2
    else:
        xmean, ymean = x.mean(0), y.mean(0)
        u, _, vt = np.linalg.svd((x - xmean).T @ (y - ymean))
        r = vt.T @ np.diag([1., 1., np.linalg.det(vt.T @ u.T)]) @ u.T
    return r, ymean - r @ xmean


def transform(x, tf):
    return x @ tf[0].T + tf[1]


def reference_pairs(traj, gt, mask):
    idx, ok = nearest(traj[:, 0], gt, MAX_REFERENCE_DT)
    selected = mask & ok
    return traj[selected, 1:4], gt[idx[selected], 1:4]


def sample_pcd(path, stride=20):
    pts = []
    with path.open() as f:
        for line in f:
            if line.startswith('DATA '):
                if line.strip() != 'DATA ascii':
                    raise ValueError('PCD is not ASCII: ' + str(path))
                break
        for i, line in enumerate(f):
            if i % stride:
                continue
            p = np.fromstring(line, sep=' ', count=3)
            if p.size == 3 and np.isfinite(p).all() and np.linalg.norm(p) > .01:
                pts.append(p)
    return np.asarray(pts)


def voxel(points, size=.5):
    keys = np.floor(points / size).astype('int64')
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[idx]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('isolated_log', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--joint-log', type=Path, required=True, help='log directory of a normal-communication run')
    parser.add_argument('--gt-dir', type=Path, default=GT_DIR)
    parser.add_argument('--input-dir', type=Path, default=INPUT_DIR)
    parser.add_argument('--candidate-shared-frame', action='store_true',
                        help='Candidate is a collaborative interruption run, not three isolated local frames')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    refs = [np.loadtxt(args.gt_dir / f'{name}_gt.txt') for name in ASSIGNMENT]
    raw = [np.loadtxt(args.input_dir / f'robot_{i}_input.tum') for i in range(3)]
    isolated = [np.loadtxt(args.isolated_log / f'robot_{i}.tum') for i in range(3)]
    joint = [np.loadtxt(args.joint_log / f'robot_{i}.tum') for i in range(3)]
    calibration_end = T0 + CALIBRATION_S
    local_tf = []
    calibration_stats = []
    for i in range(3):
        x, y = reference_pairs(raw[i], refs[i], raw[i][:, 0] <= calibration_end)
        if len(x) < 30 or np.ptp(x[:, :2], axis=0).max() < 5:
            raise ValueError(f'Insufficient early calibration for robot {i}')
        tf = transform_fit(x, y, planar=True)
        local_tf.append(tf)
        calibration_stats.append({'robot': i, 'points': len(x), 'duration_s': CALIBRATION_S,
                                  'planar_rmse_m': float(np.sqrt(np.mean(np.sum((transform(x, tf)[:, :2]-y[:, :2])**2, axis=1))))})
    pairs = [reference_pairs(joint[i], refs[i], joint[i][:, 0] <= calibration_end) for i in range(3)]
    joint_tf = transform_fit(np.concatenate([x for x, _ in pairs]), np.concatenate([y for _, y in pairs]))
    def pooled_calibration_rmse(items, tf):
        err = np.concatenate([transform(x, tf) - y for x, y in items])
        return float(np.sqrt(np.mean(np.sum(err * err, axis=1))))
    joint_calibration_rmse = pooled_calibration_rmse(pairs, joint_tf)
    candidate_tf = None
    candidate_calibration_rmse = None
    if args.candidate_shared_frame:
        pairs = [reference_pairs(isolated[i], refs[i], isolated[i][:, 0] <= calibration_end) for i in range(3)]
        candidate_tf = transform_fit(np.concatenate([x for x, _ in pairs]), np.concatenate([y for _, y in pairs]))
        candidate_calibration_rmse = pooled_calibration_rmse(pairs, candidate_tf)
        if candidate_calibration_rmse > 30:
            raise ValueError(f'Candidate frames may not share a coordinate system: initial pooled RMSE {candidate_calibration_rmse:.1f} m')

    # Evaluate every method at the same collaborative keyframe times, using only
    # the post-calibration part with reference and all estimates nearby.
    records = []
    aligned = {'input': [], 'isolated': [], 'joint': [], 'reference': []}
    evaluation_times = []
    for i in range(3):
        times = joint[i][:, 0]
        ia, oka = nearest(times, raw[i], MAX_TRAJECTORY_DT)
        ib, okb = nearest(times, isolated[i], MAX_TRAJECTORY_DT)
        ig, okg = nearest(times, refs[i], MAX_REFERENCE_DT)
        keep = (times > calibration_end) & oka & okb & okg
        if keep.sum() < 30:
            raise ValueError(f'Insufficient held-out common points for robot {i}: {keep.sum()}')
        gt = refs[i][ig[keep], 1:4]
        evaluation_times.append(times[keep])
        aligned['input'].append(transform(raw[i][ia[keep], 1:4], local_tf[i]))
        aligned['isolated'].append(transform(isolated[i][ib[keep], 1:4], candidate_tf if candidate_tf is not None else local_tf[i]))
        aligned['joint'].append(transform(joint[i][keep, 1:4], joint_tf))
        aligned['reference'].append(gt)
        for mode in ('input', 'isolated', 'joint'):
            e = np.linalg.norm(aligned[mode][i] - gt, axis=1)
            name = 'interruption' if mode == 'isolated' and args.candidate_shared_frame else mode
            records.append({'mode': name, 'robot': i, 'samples': len(e), 'rmse_m': float(np.sqrt(np.mean(e**2))),
                            'median_m': float(np.median(e)), 'p90_m': float(np.quantile(e, .9))})
    for mode in ('input', 'isolated', 'joint'):
        e = np.concatenate([np.linalg.norm(aligned[mode][i]-aligned['reference'][i], axis=1) for i in range(3)])
        name = 'interruption' if mode == 'isolated' and args.candidate_shared_frame else mode
        records.append({'mode': name, 'robot': 'all', 'samples': len(e), 'rmse_m': float(np.sqrt(np.mean(e**2))),
                        'median_m': float(np.median(e)), 'p90_m': float(np.quantile(e, .9))})
    with (args.out / 'trajectory_metrics.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=records[0].keys()); w.writeheader(); w.writerows(records)

    origin = aligned['reference'][0][0, :2]
    colors = ('#dc2626', '#2563eb', '#16a34a')
    fig, axes = plt.subplots(1, 3, figsize=(17, 6), constrained_layout=True, sharex=True, sharey=True)
    candidate_label = 'Временный обрыв' if args.candidate_shared_frame else 'Без связи'
    for ax, mode, title in zip(axes, ('input', 'isolated', 'joint'), ('Входная одометрия', candidate_label, 'Совместный результат')):
        for i in range(3):
            p = aligned[mode][i][:, :2] - origin
            ref = aligned['reference'][i][:, :2] - origin
            ax.scatter(ref[:, 0], ref[:, 1], color=colors[i], s=2, alpha=.45, rasterized=True)
            # Do not draw a straight path across gaps in RTK coverage.
            segments = np.split(p, np.where(np.diff(evaluation_times[i]) > 5.)[0] + 1)
            for k, segment in enumerate(segments):
                ax.plot(segment[:, 0], segment[:, 1], color=colors[i], lw=1.3, label=f'Робот {i}' if k == 0 else None)
        ax.set_title(title); ax.set_xlabel('x относительно общей точки RTK, м')
        ax.set_aspect('equal', adjustable='box'); ax.grid(alpha=.2)
    axes[0].set_ylabel('y относительно общей точки RTK, м')
    axes[-1].legend(fontsize=8)
    fig.suptitle('Одни и те же оценочные точки после первых 100 с; точки — RTK')
    fig.savefig(args.out / 'trajectory_comparison.png', dpi=160); plt.close(fig)

    # Convert all maps into the saved joint-map coordinate system. No ICP or
    # map-to-map registration is used. The ROI is fixed from part 2 (-66, 62).
    def joint_frame(world):
        return (world - joint_tf[1]) @ joint_tf[0]
    maps = {'joint': [], 'isolated': []}
    for i in range(3):
        maps['joint'].append(sample_pcd(args.joint_log / f'robot_{i}_pcd_wrt_global.pcd'))
        p = sample_pcd(args.isolated_log / f'robot_{i}_pcd_wrt_global.pcd')
        maps['isolated'].append(joint_frame(transform(p, candidate_tf if candidate_tf is not None else local_tf[i])))
    roi = (-86., -46., 42., 82.)
    surface = []
    for mode in ('joint', 'isolated'):
        region = []
        for p in maps[mode]:
            keep = (p[:, 0]>=roi[0]) & (p[:, 0]<=roi[1]) & (p[:, 1]>=roi[2]) & (p[:, 1]<=roi[3])
            region.append(voxel(p[keep]))
        for i, j in ((0, 1), (0, 2), (1, 2)):
            if min(len(region[i]), len(region[j])) < 5:
                raise ValueError('Insufficient map points in ROI')
            d1 = cKDTree(region[j]).query(region[i], workers=1)[0]
            d2 = cKDTree(region[i]).query(region[j], workers=1)[0]
            for a, b, d in ((i, j, d1), (j, i, d2)):
                name = 'interruption' if mode == 'isolated' and args.candidate_shared_frame else mode
                surface.append({'mode': name, 'source': a, 'target': b, 'source_voxels': len(d),
                                'median_m': float(np.median(d)), 'p90_m': float(np.quantile(d, .9)),
                                'within_0_5m_fraction': float(np.mean(d <= .5))})
    with (args.out / 'surface_metrics.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=surface[0].keys()); w.writeheader(); w.writerows(surface)

    fig, axes = plt.subplots(1, 2, figsize=(13, 6), constrained_layout=True, sharex=True, sharey=True)
    candidate_label = 'Временный обрыв канала данных' if args.candidate_shared_frame else 'Без межроботной связи'
    for ax, mode, label in zip(axes, ('isolated', 'joint'), (candidate_label, 'Совместная оптимизация')):
        for i, p in enumerate(maps[mode]):
            keep = (p[:, 0]>=roi[0]) & (p[:, 0]<=roi[1]) & (p[:, 1]>=roi[2]) & (p[:, 1]<=roi[3])
            ax.scatter(p[keep, 0], p[keep, 1], s=.4, alpha=.25, c=colors[i], label=f'Робот {i}', rasterized=True)
        ax.set(xlim=roi[:2], ylim=roi[2:], title=label, xlabel='x, м', ylabel='y, м')
        ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend(markerscale=7)
    fig.suptitle('Одинаковый участок 40 × 40 м; привязка по первым 100 с, без подгонки карт')
    fig.savefig(args.out / 'surface_overlap.png', dpi=170); plt.close(fig)
    provenance = {'input_note': 'Co-LRIO-prepared odometry; upstream independence not independently verified',
                  'mapping_inferred': list(ASSIGNMENT), 'calibration_start_stamp': T0,
                  'calibration_duration_s': CALIBRATION_S, 'local_calibration': calibration_stats,
                  'local_transforms_R_t': [{'R': r.tolist(), 't': t.tolist()} for r, t in local_tf],
                  'joint_transform_R_t': {'R': joint_tf[0].tolist(), 't': joint_tf[1].tolist()},
                  'joint_calibration_rmse_m': joint_calibration_rmse,
                  'candidate_shared_transform_R_t': {'R': candidate_tf[0].tolist(), 't': candidate_tf[1].tolist()} if candidate_tf is not None else None,
                  'candidate_calibration_rmse_m': candidate_calibration_rmse,
                  'association_max_delta_reference_s': MAX_REFERENCE_DT,
                  'association_max_delta_trajectories_s': MAX_TRAJECTORY_DT,
                  'roi_joint_frame_xy_m': roi, 'pcd_sample_stride': 20, 'voxel_size_m': .5,
                  'warning': 'Surface NN distance conflates visibility, sparse sampling and correspondence, not absolute accuracy.'}
    (args.out / 'analysis_parameters.json').write_text(json.dumps(provenance, indent=2))
    print('calibration:', calibration_stats)
    print('trajectory:', records)
    print('surface:', surface)


if __name__ == '__main__':
    main()
