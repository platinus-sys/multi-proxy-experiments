#!/usr/bin/env python3
"""Measure local keyframe position stability from an existing marker rosbag.

Runs inside ROS Noetic. This is an observation of published marker positions,
not proof that all callbacks, network queues or global optimization finished.
"""
import json
import math
import sys
import rosbag

bag_path, meta_path, output_path = sys.argv[1:4]
threshold_m = 0.05
required_quiet_s = 10.
max_gap_s = 4.
with open(meta_path) as f:
    meta = dict(line.rstrip('\n').split('=', 1) for line in f if '=' in line)
play_start = float(meta['play_start_epoch'])
play_end = float(meta['play_end_epoch'])
save_request = float(meta['save_request_epoch'])
topics = ['/robot_%d/key_frame' % i for i in range(3)]
tf_topics = ['/tf', '/tf_static']
state = [{'anchor': None, 'first': None, 'last': None, 'last_change': None,
          'changes': 0, 'samples': 0, 'max_gap_s': 0., 'final_keyframes': 0}
         for _ in range(3)]
tf_state = [{'anchor': None, 'first': None, 'last': None, 'last_change': None,
             'changes': 0, 'samples': 0, 'max_gap_s': 0.} for _ in range(3)]

with rosbag.Bag(bag_path) as bag:
    for topic, msg, stamp in bag.read_messages(topics=topics + tf_topics):
        t = stamp.to_sec()
        if t < play_end or t >= save_request:
            continue
        if topic in tf_topics:
            for tr in msg.transforms:
                name = tr.child_frame_id.lstrip('/')
                if name not in ['robot_%d/map' % i for i in range(3)]:
                    continue
                robot = int(name.split('/')[0][-1])
                s = tf_state[robot]
                if s['last'] is not None:
                    s['max_gap_s'] = max(s['max_gap_s'], t - s['last'])
                else:
                    s['first'] = t
                s['last'] = t
                s['samples'] += 1
                v, q = tr.transform.translation, tr.transform.rotation
                sample = (tr.header.frame_id, (v.x, v.y, v.z), (q.x, q.y, q.z, q.w))
                anchor = s['anchor']
                changed = anchor is None or sample[0] != anchor[0]
                if anchor is not None and not changed:
                    a, b = sample[2], anchor[2]
                    na = math.sqrt(sum(x*x for x in a)); nb = math.sqrt(sum(x*x for x in b))
                    if na == 0 or nb == 0:
                        changed = True
                    else:
                        dot = min(1., abs(sum(x*y for x, y in zip(a, b)) / (na*nb)))
                        changed = math.dist(sample[1], anchor[1]) >= threshold_m or 2 * math.acos(dot) >= math.radians(.5)
                if changed:
                    s['anchor'] = sample
                    s['last_change'] = t
                    s['changes'] += 1
            continue
        robot = topics.index(topic)
        positions = {}
        for marker in msg.markers:
            if marker.ns.endswith('_Key_frame_ids') and marker.type == marker.TEXT_VIEW_FACING:
                p = marker.pose.position
                positions[marker.id] = (p.x, p.y, p.z)
        if not positions:
            continue
        s = state[robot]
        if s['last'] is not None:
            s['max_gap_s'] = max(s['max_gap_s'], t - s['last'])
        else:
            s['first'] = t
        s['last'] = t
        s['samples'] += 1
        s['final_keyframes'] = len(positions)
        anchor = s['anchor']
        changed = anchor is None or positions.keys() != anchor.keys()
        if anchor is not None and not changed:
            changed = any(math.dist(positions[k], anchor[k]) >= threshold_m for k in positions)
        if changed:
            s['anchor'] = positions
            s['last_change'] = t
            s['changes'] += 1

for i, s in enumerate(state):
    s.pop('anchor')
    s['robot'] = i
    s['quiet_at_end_s'] = None if s['last_change'] is None else s['last'] - s['last_change']
    s['observed_stable'] = bool(s['samples'] >= 2 and s['max_gap_s'] <= max_gap_s and
                                s['quiet_at_end_s'] is not None and s['quiet_at_end_s'] >= required_quiet_s)
    s['observed_stable_after_play_s'] = (s['last_change'] - play_end + required_quiet_s) if s['observed_stable'] else None

for i, s in enumerate(tf_state):
    s.pop('anchor')
    s['robot'] = i
    s['quiet_at_end_s'] = None if s['last_change'] is None else s['last'] - s['last_change']
    s['observed_stable'] = bool(s['samples'] >= 2 and s['max_gap_s'] <= max_gap_s and
                                s['quiet_at_end_s'] is not None and s['quiet_at_end_s'] >= required_quiet_s)

success = all(s['observed_stable'] for s in state)
last = max(s['last_change'] for s in state if s['last_change'] is not None) if any(s['last_change'] is not None for s in state) else None
combined_success = success and all(s['observed_stable'] for s in tf_state)
combined_last = max([s['last_change'] for s in state + tf_state]) if combined_success else None
result = {'metric': 'observed_local_keyframe_position_stability', 'threshold_m': threshold_m,
          'required_quiet_s': required_quiet_s, 'max_allowed_marker_gap_s': max_gap_s,
          'play_start_epoch': play_start, 'play_end_epoch': play_end, 'save_request_epoch': save_request,
          'observation_window_s': save_request - play_end, 'stable': success,
          'tf_recorded_for_all_robots': all(s['samples'] > 0 for s in tf_state),
          'local_and_tf_observed_stable': combined_success,
          'wall_time_from_play_start_to_local_and_tf_stable_s': (combined_last - play_start + required_quiet_s) if combined_success else None,
          'wall_time_after_play_to_local_and_tf_stable_s': (combined_last - play_end + required_quiet_s) if combined_success else None,
          'tf_rotation_threshold_deg': .5,
          'wall_time_from_play_start_to_observed_stability_s': (last - play_start + required_quiet_s) if success else None,
          'wall_time_after_play_to_observed_stability_s': (last - play_end + required_quiet_s) if success else None,
          'limitations': 'Monitors local keyframe marker positions/IDs and, if available, robot-map TF translation/orientation. Does not observe individual keyframe orientations, in-flight network work, optimization queues, dropped markers, or delayed future changes. First observation is an anchor, not necessarily a change. Results apply only to recorded pre-save window.',
          'robots': state, 'robot_map_tf': tf_state}
with open(output_path, 'w') as f:
    json.dump(result, f, indent=2)
print(json.dumps(result, indent=2))
