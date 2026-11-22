#!/usr/bin/env python3
"""Summarize measured optimization-loop wall durations during bag playback."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('session', type=Path)
args = parser.parse_args()
rows = []
for run in sorted(args.session.glob('*/run_*')):
    if not (run / 'COMPLETE.txt').exists():
        continue
    meta = dict(line.split('=', 1) for line in (run / 'metadata.txt').read_text().splitlines() if '=' in line)
    lo, hi = float(meta['play_start_epoch']), float(meta['play_end_epoch'])
    all_values = []
    for robot in range(3):
        path = run / 'log' / f'robot_{robot}_optimization_timing.csv'
        with path.open(newline='') as f:
            reader = csv.DictReader(f)
            data = [float(r['elapsed_ms']) for r in reader if float(r['wall_start_epoch']) >= lo and float(r['wall_end_epoch']) <= hi]
        if not data or not np.isfinite(data).all() or min(data) < 0:
            raise ValueError(f'Empty or invalid timing: {path}')
        arr = np.array(data)
        all_values.extend(data)
        rows.append({'mode': meta['mode'], 'repeat': int(meta['repeat']), 'robot': robot,
                     'iterations': len(arr), 'sum_iteration_wall_s': arr.sum() / 1000.,
                     'mean_iteration_ms': arr.mean(), 'median_iteration_ms': np.median(arr),
                     'p95_iteration_ms': np.quantile(arr, .95), 'max_iteration_ms': arr.max(),
                     'playback_wall_s': hi - lo})
    arr = np.array(all_values)
    rows.append({'mode': meta['mode'], 'repeat': int(meta['repeat']), 'robot': 'all',
                 'iterations': len(arr), 'sum_iteration_wall_s': arr.sum() / 1000.,
                 'mean_iteration_ms': arr.mean(), 'median_iteration_ms': np.median(arr),
                 'p95_iteration_ms': np.quantile(arr, .95), 'max_iteration_ms': arr.max(),
                 'playback_wall_s': hi-lo})
if not rows:
    raise ValueError('No complete runs found')
with (args.session / 'optimization_timing_summary.csv').open('w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=rows[0].keys()); w.writeheader(); w.writerows(rows)
(args.session / 'optimization_timing_method.json').write_text(json.dumps({
    'interval': 'only complete iterations whose start and end are between play_start_epoch and play_end_epoch',
    'clock': 'std::chrono::steady_clock for iteration elapsed; ros::WallTime for boundaries',
    'includes': 'one globalOptimizationThread iteration after rate.sleep, including iPCM, graph updates, GTSAM optimization, dual processing and waiting for locks',
    'excludes': 'rate.sleep, descriptor/candidate/ICP worker threads, ROS callbacks, map rendering, communication latency outside this thread, save service, and post-play iterations',
    'note': 'Summed durations across three concurrent workers are worker-wall-seconds, not end-to-end elapsed time or CPU seconds. The solver continues periodic updates after playback, so this does not establish a completion/convergence time.'}, indent=2))
for row in rows:
    if row['robot'] == 'all':
        print(f"{row['mode']} repeat={row['repeat']} iter={row['iterations']} sum_worker_s={row['sum_iteration_wall_s']:.3f} mean_ms={row['mean_iteration_ms']:.3f} p95_ms={row['p95_iteration_ms']:.3f}")
