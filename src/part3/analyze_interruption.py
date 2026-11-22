#!/usr/bin/env python3
"""Summarize observed relay blocking and resumption, without inferring convergence."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('run', type=Path)
args = parser.parse_args()
run = args.run
def readkv(path):
    return dict(line.split('=', 1) for line in path.read_text().splitlines() if '=' in line)
events = {k: float(v) for k, v in readkv(run / 'outage_events.txt').items()}
start = float(readkv(run / 'metadata.txt')['play_start_epoch'])
with (run / 'bridge_counts.csv').open(newline='') as f:
    rows = [{k: float(v) if k == 'epoch' else int(v) for k, v in row.items()} for row in csv.DictReader(f)]
if not rows:
    raise ValueError('No relay samples')
cats = ('descriptor_info', 'candidate_info', 'verified_loop', 'dual_info')
blocked = [r for r in rows if r['blocked']]
if not blocked:
    raise ValueError('No blocked samples were observed')
start_block = blocked[0]['epoch']
end_block = next((r['epoch'] for r in rows if r['epoch'] > blocked[-1]['epoch'] and not r['blocked']), None)
if end_block is None:
    raise ValueError('No unblocked sample observed after outage')
out = {'play_start_epoch': start, 'block_requested_epoch': events['block_requested_epoch'],
       'restore_requested_epoch': events['restore_requested_epoch'], 'first_blocked_sample_epoch': start_block,
       'first_unblocked_sample_epoch': end_block, 'blocked_observed_seconds_approx': end_block - start_block,
       'note': 'Counters count relay callback events, not delivery or optimization convergence; heartbeat remained active.'}
for cat in cats:
    forward_key, blocked_key = cat + '_forwarded', cat + '_blocked'
    after = [r for r in rows if r['epoch'] > end_block and r[forward_key] > 0]
    during = [r for r in rows if start_block < r['epoch'] <= end_block]
    out[cat] = {'forwarded_before': sum(r[forward_key] for r in rows if r['epoch'] <= start_block),
                'blocked_during': sum(r[blocked_key] for r in during),
                'forwarded_during': sum(r[forward_key] for r in during),
                'forwarded_after': sum(r[forward_key] for r in rows if r['epoch'] > end_block),
                'first_forward_after_unblocked_s': round(after[0]['epoch'] - end_block, 3) if after else None}
(run / 'outage_summary.json').write_text(json.dumps(out, indent=2, ensure_ascii=False))
fig, axes = plt.subplots(4, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
for ax, cat in zip(axes, cats):
    x = [r['epoch'] - start for r in rows]
    ax.plot(x, [r[cat + '_forwarded'] for r in rows], lw=.8, color='#2563eb', label='Пропущено')
    ax.plot(x, [r[cat + '_blocked'] for r in rows], lw=.8, color='#dc2626', label='Заблокировано')
    ax.axvspan(start_block-start, end_block-start, color='orange', alpha=.2)
    ax.set_ylabel(cat); ax.grid(alpha=.2)
axes[0].legend(loc='upper right'); axes[-1].set_xlabel('Настенное время с начала rosbag, с')
fig.suptitle('Обрыв канала данных: события ретранслятора, сообщений/с')
fig.savefig(run / 'outage_counts.png', dpi=160); plt.close(fig)
print(json.dumps(out, indent=2, ensure_ascii=False))
