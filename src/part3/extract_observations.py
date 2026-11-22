#!/usr/bin/env python3
"""Extract time-series evidence from ROS marker and TF bag files."""
import csv
import sys
import rosbag

marker_bag, tf_bag, out_dir = sys.argv[1:]
with open(out_dir + '/loop_markers.csv', 'w') as f, open(out_dir + '/keyframe_snapshots.csv', 'w') as snapshots:
    w = csv.writer(f)
    sw = csv.writer(snapshots)
    w.writerow(['bag_epoch', 'header_stamp', 'robot', 'reference_frame', 'marker_line_segments'])
    sw.writerow(['bag_epoch', 'robot', 'frame_id', 'keyframe_id', 'x', 'y', 'z'])
    last_snapshot = {}
    with rosbag.Bag(marker_bag) as b:
        for topic, msg, stamp in b.read_messages():
            if topic.endswith('/key_frame'):
                robot = topic.split('/')[1]
                now = stamp.to_sec()
                if now - last_snapshot.get(robot, 0) < 5:
                    continue
                last_snapshot[robot] = now
                for marker in msg.markers:
                    if marker.ns.endswith('_Key_frame_ids') and marker.type == marker.TEXT_VIEW_FACING:
                        p = marker.pose.position
                        sw.writerow([now, robot, marker.header.frame_id, marker.id, p.x, p.y, p.z - .5])
                continue
            if not topic.endswith('/loop_closure_constraints'):
                continue
            for marker in msg.markers:
                if marker.ns.endswith('_loop_frame_edges'):
                    w.writerow([stamp.to_sec(), marker.header.stamp.to_sec(), topic.split('/')[1],
                                marker.header.frame_id, len(marker.points) // 2])
with open(out_dir + '/transforms.csv', 'w') as f:
    w = csv.writer(f)
    w.writerow(['bag_epoch', 'header_stamp', 'parent', 'child', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
    with rosbag.Bag(tf_bag) as b:
        for _, msg, stamp in b.read_messages(topics=['/tf', '/tf_static']):
            for tr in msg.transforms:
                t, q = tr.transform.translation, tr.transform.rotation
                w.writerow([stamp.to_sec(), tr.header.stamp.to_sec(), tr.header.frame_id, tr.child_frame_id,
                            t.x, t.y, t.z, q.x, q.y, q.z, q.w])
