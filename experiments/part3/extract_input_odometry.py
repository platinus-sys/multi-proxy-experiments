#!/usr/bin/env python3
"""Extract the per-robot front-end odometry from College as TUM trajectories.

Run in the ROS Noetic container: python3 - /datasets/...bag /output/directory
"""
import os
import sys
import rosbag

bag_path, out_dir = sys.argv[1:]
os.makedirs(out_dir, exist_ok=True)
files = [open(os.path.join(out_dir, 'robot_%d_input.tum' % i), 'w') for i in range(3)]
topics = ['/robot_%d/mapping/odometry' % i for i in range(3)]
counts = [0, 0, 0]
try:
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, _ in bag.read_messages(topics=topics):
            i = topics.index(topic)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            files[i].write('%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n' %
                           (msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            counts[i] += 1
finally:
    for f in files:
        f.close()
print('Odometry messages per robot:', counts)
