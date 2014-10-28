#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped


pub = rospy.Publisher('/gateway/pose_stamped', PoseStamped, queue_size = 10)


def callback(data):
    p = PoseStamped()
    p.header = data.header
    p.pose.position = data.transform.translation
    p.pose.orientation = data.transform.rotation

    pub.publish(p);

def gateway():

    rospy.init_node('gateway', anonymous=True)
    rospy.Subscriber("/gateway/transform_stamped", TransformStamped, callback)

    rospy.spin()
        
if __name__ == '__main__':
    gateway()
