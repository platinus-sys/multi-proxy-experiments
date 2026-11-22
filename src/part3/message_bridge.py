#!/usr/bin/env python3
"""Relay four inter-robot topics, with a switchable data-channel interruption.

Heartbeat remains connected; therefore this is a data-channel interruption,
not a full network partition. Counts indicate relay events, not delivery.
"""
import csv
import os
import sys
import threading
import time

import rospy
from std_msgs.msg import Int8MultiArray

out_dir = sys.argv[1]
os.makedirs(out_dir, exist_ok=True)
rospy.init_node('part3_message_bridge', anonymous=False, disable_signals=False)
fields = ('epoch', 'blocked', 'descriptor_info_forwarded', 'descriptor_info_blocked',
          'candidate_info_forwarded', 'candidate_info_blocked', 'verified_loop_forwarded',
          'verified_loop_blocked', 'dual_info_forwarded', 'dual_info_blocked')
f = open(os.path.join(out_dir, 'bridge_counts.csv'), 'w', buffering=1)
writer = csv.DictWriter(f, fieldnames=fields)
writer.writeheader()
lock = threading.Lock()
state = {'blocked': False, 'counts': {field: 0 for field in fields[2:]}}
publishers = []
subscribers = []


def callback(msg, pair):
    category, publisher = pair
    with lock:
        blocked = state['blocked']
        key = category + ('_blocked' if blocked else '_forwarded')
        state['counts'][key] += 1
        if not blocked:
            publisher.publish(msg)


for robot in range(3):
    for category in ('descriptor_info', 'candidate_info', 'verified_loop', 'dual_info'):
        suffix = '/robot_%d/loop/%s' % (robot, category)
        pub = rospy.Publisher('/part3/rx' + suffix, Int8MultiArray, queue_size=100)
        publishers.append(pub)
        subscribers.append(rospy.Subscriber('/part3/tx' + suffix, Int8MultiArray, callback,
                                             callback_args=(category, pub), queue_size=100))


def tick(_):
    blocked = bool(rospy.get_param('/part3/block_interrobot_data', False))
    with lock:
        state['blocked'] = blocked
        row = {'epoch': '%.6f' % time.time(), 'blocked': int(blocked)}
        row.update(state['counts'])
        writer.writerow(row)
        state['counts'] = {field: 0 for field in fields[2:]}


timer = rospy.Timer(rospy.Duration(1.0), tick)
rospy.loginfo('Inter-robot relay ready; heartbeat remains unmodified')
try:
    rospy.spin()
finally:
    timer.shutdown()
    f.close()
