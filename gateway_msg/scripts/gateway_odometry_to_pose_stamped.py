#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


pub = rospy.Publisher('/gateway/pose_stamped', PoseStamped, queue_size = 10)


def callback(data):
    p = PoseStamped()
    p.header = data.header
    p.pose = data.pose.pose

    pub.publish(p);

def gateway():

    rospy.init_node('gateway_odometry_to_pose_stamped', anonymous=True)
    rospy.Subscriber("/odom", Odometry, callback)

    rospy.spin()
        
if __name__ == '__main__':
    gateway()
