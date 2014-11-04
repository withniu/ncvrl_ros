#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path


import rosbag
import csv

bag = rosbag.Bag('sfm_pose.bag', 'w')

stamp_ref = 1413303981.13749303
index_ref = 70
path = Path()

index = 0
try:
    with open('localization.csv', 'rb') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in csvreader:
#            print row
            header = Header()
            #header.seq = int(row[0])
            header.seq = index
            index = index + 1
            stamp = stamp_ref + (float(row[0]) - index_ref) * 1.0 / 48
            header.stamp.secs = int(stamp);
            header.stamp.nsecs = int((stamp - int(stamp)) * 1000000000)
            print header.stamp.secs
            header.frame_id = '/world'
            ps = PoseStamped()
            ps.header = header

            p = Pose()
            p.position.x = float(row[1])
            p.position.y = float(row[2])
            p.position.z = float(row[3])
            
            p.orientation.x = float(row[4])
            p.orientation.y = float(row[5])
            p.orientation.z = float(row[6])
            p.orientation.w = float(row[7])

            ps.pose = p
            path.header = header
            path.poses.append(ps)
            if header.seq % 100 == 0:
                bag.write('sfm_path', path, header.stamp)

            bag.write('sfm_pose', ps, header.stamp)
finally:
    bag.close()
