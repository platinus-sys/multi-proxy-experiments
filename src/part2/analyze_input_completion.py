#!/usr/bin/env python3
"""Measure when the final expected cloud/odom pair appeared in node logs."""
import argparse
import csv
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('session', type=Path)
args = parser.parse_args()
reference = json.loads((ROOT / 'results/part2/college_input_last_stamps.json').read_text())
records = []
for run in sorted(args.session.glob('*/run_*')):
    if not (run / 'COMPLETE.txt').exists():
        continue
    meta = dict(line.split('=', 1) for line in (run / 'metadata.txt').read_text().splitlines() if '=' in line)
    start = float(meta['play_start_epoch'])
    end = float(meta['play_end_epoch'])
    found = []
    for i in range(3):
        expected = reference[i]['last_common_sensor_stamp_ns'] / 1e9
        log = (run / 'log' / f'robot_{i}_init_descriptor.txt').read_text()
        blocks = re.findall(r'Time:\s*([\d.]+)\s*\nframe_id:\s*(\d+)\s*\ntime:\s*([\d.]+)', log)
        matches = [(float(wall), int(frame), float(stamp)) for wall, frame, stamp in blocks if abs(float(stamp)-expected) < 1e-6]
        last_wall = matches[-1][0] if matches else None
        found.append(last_wall)
        records.append({'mode': meta['mode'], 'repeat': int(meta['repeat']), 'robot': i,
                        'expected_last_pair_sensor_stamp': expected,
                        'input_last_pair_bag_offset_s': reference[i]['last_common_bag_offset_s'],
                        'matching_log_entries': len(matches),
                        'last_pair_logged_after_play_start_s': None if last_wall is None else last_wall-start,
                        'last_pair_logged_before_play_end_s': None if last_wall is None else end-last_wall,
                        'frame_log_entries': len(blocks), 'input_cloud_messages': reference[i]['message_counts']['cloud']})
    if all(v is not None for v in found):
        records.append({'mode': meta['mode'], 'repeat': int(meta['repeat']), 'robot': 'all',
                        'expected_last_pair_sensor_stamp': None, 'input_last_pair_bag_offset_s': None,
                        'matching_log_entries': None, 'last_pair_logged_after_play_start_s': max(found)-start,
                        'last_pair_logged_before_play_end_s': end-max(found), 'frame_log_entries': None,
                        'input_cloud_messages': None})
if not records:
    raise ValueError('No complete runs')
with (args.session / 'input_completion_summary.csv').open('w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=records[0].keys()); w.writeheader(); w.writerows(records)
for row in records:
    print(row)
