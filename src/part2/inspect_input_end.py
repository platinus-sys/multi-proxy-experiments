#!/usr/bin/env python3
"""Find the last timestamp shared by each robot's cloud and odometry."""
import json
import sys
import bisect
import rosbag

bag_path = sys.argv[1]
topics = {f'/robot_{i}/{suffix}': (i, kind) for i in range(3)
          for suffix, kind in (('mapping/odometry', 'odom'), ('deskew/cloud_deskewed', 'cloud'))}
stamps = [{k: set() for k in ('odom', 'cloud')} for _ in range(3)]
counts = [{k: 0 for k in ('odom', 'cloud')} for _ in range(3)]
cloud_record_times = [{} for _ in range(3)]
with rosbag.Bag(bag_path) as bag:
    bag_start = bag.get_start_time()
    for topic, msg, bag_time in bag.read_messages(topics=list(topics)):
        robot, kind = topics[topic]
        ns = msg.header.stamp.to_nsec()
        stamps[robot][kind].add(ns)
        if kind == 'cloud':
            cloud_record_times[robot][ns] = bag_time.to_sec()
        counts[robot][kind] += 1
out = []
for robot in range(3):
    odom = sorted(stamps[robot]['odom'])
    common = []
    for cloud in stamps[robot]['cloud']:
        idx = bisect.bisect_left(odom, cloud)
        candidates = odom[max(0, idx-1):min(len(odom), idx+1)]
        if candidates and min(abs(x-cloud) for x in candidates) < 1000:
            common.append(cloud)
    if not common:
        raise ValueError('No cloud/odometry timestamps within 1 microsecond for robot %d' % robot)
    out.append({'robot': robot, 'message_counts': counts[robot],
                'last_common_sensor_stamp_ns': max(common),
                'last_common_bag_record_epoch': cloud_record_times[robot][max(common)],
                'last_common_bag_offset_s': cloud_record_times[robot][max(common)] - bag_start,
                'last_cloud_sensor_stamp_ns': max(stamps[robot]['cloud']),
                'last_odom_sensor_stamp_ns': max(stamps[robot]['odom']),
                'unique_pairs_within_1us': len(common)})
print(json.dumps(out, indent=2))
