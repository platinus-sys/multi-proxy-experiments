#!/usr/bin/env python3
"""Provisional shared-SE(3) ATE and recorded traffic/timing comparison."""
import csv
import itertools
import argparse
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--session', type=Path, required=True, help='Results session containing baseline/delay/loss runs')
parser.add_argument('--gt-dir', type=Path, default=ROOT / 'datasets/S3E/ground_truth_campus_road_1')
args = parser.parse_args()
SESSION = args.session
GT_DIR = args.gt_dir
NAMES = ('alpha', 'bob', 'carol')
MAX_DT = 0.55  # Nearest reference sample; never interpolate across RTK gaps.


def matches(estimate, reference):
    t = estimate[:, 0]
    j = np.searchsorted(reference[:, 0], t)
    lo = np.clip(j - 1, 0, len(reference) - 1)
    hi = np.clip(j, 0, len(reference) - 1)
    choose_hi = np.abs(reference[hi, 0] - t) < np.abs(reference[lo, 0] - t)
    idx = np.where(choose_hi, hi, lo)
    mask = np.abs(reference[idx, 0] - t) <= MAX_DT
    return estimate[mask, 1:4], reference[idx[mask], 1:4]


def align(pairs):
    x = np.concatenate([a for a, _ in pairs])
    y = np.concatenate([b for _, b in pairs])
    xmean, ymean = x.mean(axis=0), y.mean(axis=0)
    u, _, vt = np.linalg.svd((x - xmean).T @ (y - ymean))
    sign = np.linalg.det(vt.T @ u.T)
    rot = vt.T @ np.diag([1., 1., sign]) @ u.T
    trans = ymean - rot @ xmean
    residuals = [a @ rot.T + trans - b for a, b in pairs]
    rmses = [float(np.sqrt(np.mean(np.sum(r * r, axis=1)))) for r in residuals]
    all_r = np.concatenate(residuals)
    return float(np.sqrt(np.mean(np.sum(all_r * all_r, axis=1)))), rmses


gt = {name: np.loadtxt(GT_DIR / f'{name}_gt.txt') for name in NAMES}
for ref in gt.values():
    if not np.all(np.diff(ref[:, 0]) > 0):
        raise ValueError('Reference timestamps must be strictly increasing')

baseline = [np.loadtxt(SESSION / f'baseline/run_1/log/robot_{i}.tum') for i in range(3)]
ranked = []
for perm in itertools.permutations(NAMES):
    pairs = [matches(baseline[i], gt[perm[i]]) for i in range(3)]
    if min(len(a) for a, _ in pairs) < 10:
        continue
    score, _ = align(pairs)
    ranked.append((score, perm))
ranked.sort()
if not ranked:
    raise RuntimeError('No valid robot/reference assignment')
assignment = ranked[0][1]

rows = []
with (SESSION / 'summary.csv').open(newline='') as f:
    summary = list(csv.DictReader(f))
for item in summary:
    mode, repeat = item['mode'], int(item['repeat'])
    estimates = [np.loadtxt(SESSION / f'{mode}/run_{repeat}/log/robot_{i}.tum') for i in range(3)]
    pairs = [matches(estimates[i], gt[assignment[i]]) for i in range(3)]
    combined, individual = align(pairs)
    row = {'mode': mode, 'repeat': repeat, 'mapping': ','.join(assignment),
           'ate_rmse_combined_m': round(combined, 6)}
    for i in range(3):
        row[f'robot_{i}_ate_rmse_m'] = round(individual[i], 6)
        row[f'robot_{i}_matched'] = len(pairs[i][0])
        row[f'robot_{i}_total'] = len(estimates[i])
    row.update({k: item[k] for k in ('netem_bytes', 'netem_packets', 'netem_dropped_packets', 'play_wall_seconds', 'play_and_settle_wall_seconds', 'save_wall_seconds')})
    payload_total = 0
    for category in ('descriptor', 'pc', 'pose2pose', 'dual'):
        values = []
        for i in range(3):
            path = SESSION / f'{mode}/run_{repeat}/log/bw_compare/robot_{i}_{category}_bw.txt'
            for line in path.read_text().splitlines():
                fields = line.split()
                if len(fields) != 2:
                    raise ValueError(f'Unexpected line in {path}: {line}')
                values.append(int(fields[1]))
        row[f'published_{category}_bytes'] = sum(values)
        row[f'published_{category}_messages'] = len(values)
        payload_total += sum(values)
    row['published_logged_messages_bytes'] = payload_total
    rows.append(row)

with (SESSION / 'metrics_provisional.csv').open('w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows(rows)
with (SESSION / 'assignment_check.csv').open('w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['baseline_joint_rmse_m', 'robot0', 'robot1', 'robot2'])
    writer.writerows([(score, *perm) for score, perm in ranked])
print('Mapping by lowest baseline joint residual:', assignment)
print('Ranked permutations:', ranked)
for row in rows:
    print(row)
