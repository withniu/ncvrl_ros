#!/usr/bin/env python

import rosbag

bags = ['imu.bag', 'sfm_pose.bag', 'images.bag']
#bags = ['imu.bag', 'sfm_pose.bag']

with rosbag.Bag('merged.bag', 'w') as outbag:
    for bag in bags:
        for topic, msg, t in rosbag.Bag(bag).read_messages():
            outbag.write(topic, msg, msg.header.stamp)
