#!/usr/bin/env python3
"""Measure how pre-outage keyframes move relative to their last observed positions.

This is a repeatable stability diagnostic, not an error or proof of convergence.
"""
import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('run', type=Path)
args = parser.parse_args()
run = args.run
events = json.loads((run / 'outage_summary.json').read_text())
block, restore, start = (events['first_blocked_sample_epoch'], events['first_unblocked_sample_epoch'], events['play_start_epoch'])
metadata_path = run / 'metadata.txt'
metadata = dict(line.split('=', 1) for line in metadata_path.read_text().splitlines() if '=' in line) if metadata_path.exists() else {}
# The save service clears dual state. Exclude snapshots after it was requested.
save_cutoff = float(metadata.get('save_request_epoch', 'inf'))

transforms = defaultdict(list)
with (run / 'transforms.csv').open(newline='') as f:
    for row in csv.DictReader(f):
        if row['parent'].lstrip('/') != 'world':
            continue
        child = row['child'].lstrip('/')
        if child not in ('robot_0/map', 'robot_1/map', 'robot_2/map'):
            continue
        values = [float(row[k]) for k in ('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw')]
        transforms[child].append((float(row['bag_epoch']), np.array(values)))
for child in transforms:
    transforms[child].sort(key=lambda t: t[0])
if len(transforms) != 3:
    raise ValueError('Expected world transforms for all three robot map frames')

snapshots = defaultdict(lambda: defaultdict(dict))
with (run / 'keyframe_snapshots.csv').open(newline='') as f:
    for row in csv.DictReader(f):
        robot = row['robot']
        t = float(row['bag_epoch'])
        snapshots[robot][t][int(row['keyframe_id'])] = np.array([float(row[k]) for k in ('x', 'y', 'z')])


def frame_pose(robot, timestamp):
    data = transforms[robot + '/map']
    times = np.array([r[0] for r in data])
    i = np.searchsorted(times, timestamp, side='right') - 1
    if i < 0 or timestamp - times[i] > 4.:
        return None
    v = data[i][1]
    return Rotation.from_quat(v[3:]).as_matrix(), v[:3]


records = []
details = {}
for robot in ('robot_0', 'robot_1', 'robot_2'):
    data = snapshots[robot]
    valid = [t for t in sorted(data) if t < save_cutoff and frame_pose(robot, t) is not None]
    prior = [t for t in valid if t < block]
    later = [t for t in valid if t > restore]
    if not prior or not later:
        raise ValueError(f'Insufficient before/after snapshots for {robot}')
    anchor_time = prior[-1]
    final_time = later[-1]
    Rf, tf = frame_pose(robot, final_time)
    final = {k: Rf @ p + tf for k, p in data[final_time].items()}
    anchor_ids = set(data[anchor_time]) & set(final)
    if len(anchor_ids) < 10:
        raise ValueError(f'Too few persistent keyframes for {robot}: {len(anchor_ids)}')
    for t in valid:
        if t < anchor_time - 5 or t > final_time:
            continue
        R, tvec = frame_pose(robot, t)
        ids = sorted(anchor_ids & set(data[t]))
        if len(ids) < 10:
            continue
        delta = np.array([np.linalg.norm(R @ data[t][k] + tvec - final[k]) for k in ids])
        records.append({'robot': robot, 'elapsed_s': t-start, 'matches': len(ids),
                        'median_displacement_to_final_m': float(np.median(delta)),
                        'p90_displacement_to_final_m': float(np.quantile(delta, .9))})
    details[robot] = {'anchor_s': anchor_time-start, 'last_snapshot_s': final_time-start,
                      'persistent_keyframes': len(anchor_ids)}
with (run / 'snapshot_stability.csv').open('w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=records[0].keys()); w.writeheader(); w.writerows(records)
fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
for robot, color in zip(('robot_0', 'robot_1', 'robot_2'), ('#dc2626', '#2563eb', '#16a34a')):
    part = [r for r in records if r['robot'] == robot]
    ax.plot([r['elapsed_s'] for r in part], [r['median_displacement_to_final_m'] for r in part], label=robot, color=color)
ax.axvspan(block-start, restore-start, color='orange', alpha=.25, label='Канал данных заблокирован')
ax.set(xlabel='Настенное время с начала воспроизведения, с', ylabel='Медианное смещение до финального положения, м',
       title='Положение одних и тех же ключевых кадров относительно последнего снимка')
ax.grid(alpha=.2); ax.legend()
fig.savefig(run / 'snapshot_stability.png', dpi=160); plt.close(fig)
(run / 'snapshot_stability_notes.json').write_text(json.dumps({'robots': details,
    'interpretation': 'Distance to last snapshot is a retrospective stability diagnostic, not an absolute error or proof of convergence. Only world-parent TF samples are used; max age 4 s.',
    'save_request_epoch': None if not np.isfinite(save_cutoff) else save_cutoff,
    'tf_recording_start_epoch': min(v[0][0] for v in transforms.values())}, indent=2))
print(json.dumps(details, indent=2))
