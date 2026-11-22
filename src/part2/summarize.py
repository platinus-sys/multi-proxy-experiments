#!/usr/bin/env python3
"""Summarize complete runs without interpreting front-end odometry as ground truth."""
import csv
import pathlib
import re
import sys

root = pathlib.Path(sys.argv[1])
rows = []
for run in sorted(root.glob('*/run_*')):
    if not (run / 'COMPLETE.txt').exists():
        continue
    meta = dict(line.split('=', 1) for line in (run / 'metadata.txt').read_text().splitlines() if '=' in line)
    def counts(name):
        content = (run / name).read_text()
        matches = re.findall(r'qdisc netem 30:.*?\n\s*Sent\s+(\d+) bytes\s+(\d+) pkt\s+\(dropped\s+(\d+)', content, re.S)
        if len(matches) != 3:
            raise ValueError(f'{run / name}: expected 3 netem counters, got {len(matches)}')
        return [sum(int(m[i]) for m in matches) for i in range(3)]
    before, after = counts('tc_before.txt'), counts('tc_after.txt')
    rows.append({'mode': meta['mode'], 'repeat': meta['repeat'],
                 'netem_bytes': after[0] - before[0], 'netem_packets': after[1] - before[1],
                 'netem_dropped_packets': after[2] - before[2],
                 'play_wall_seconds': round(float(meta['play_end_epoch']) - float(meta['play_start_epoch']), 3),
                 'play_and_settle_wall_seconds': round(float(meta['save_request_epoch']) - float(meta['play_start_epoch']), 3),
                 'save_wall_seconds': round(float(meta['save_end_epoch']) - float(meta['save_request_epoch']), 3),
                 'ate_rmse': 'not_available_without_ground_truth'})
if not rows:
    raise SystemExit('No complete runs to summarize')
with (root / 'summary.csv').open('w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows(rows)
