#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


pub = rospy.Publisher('/gateway/pose_stamped', PoseStamped, queue_size = 10)
pub_path = rospy.Publisher('/gateway/path', Path, queue_size = 10)

path = Path()

def callback(data):
    p = PoseStamped()
    p.header = data.header
    p.pose = data.pose.pose
    path.poses.append(p)
    path.header = data.header
    if p.header.seq % 100 == 0:
        pub_path.publish(path)

    pub.publish(p);

def gateway():

    rospy.init_node('gateway', anonymous = True)
    rospy.Subscriber("/gateway/pose_with_covariance_stamped", PoseWithCovarianceStamped, callback)

    rospy.spin()
        
if __name__ == '__main__':
    gateway()
