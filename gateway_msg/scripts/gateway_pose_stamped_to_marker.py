#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


pub = rospy.Publisher('/gateway/marker', Marker, queue_size = 10)


def callback(data):
    

    m = Marker()
    m.header.frame_id = data.header.frame_id
#    m.header.stamp = rospy.get_time()
    m.ns = 'ncvrl'
    m.id = 0
    m.type = 2
#    m.pose.position.x = 0
#    m.pose.position.y = 0
#    m.pose.position.z = 0
#    m.pose.orientation.x = 0
#    m.pose.orientation.y = 0
#    m.pose.orientation.z = 0
#    m.pose.orientation.w = 1.0

    m.pose = data.pose

    m.scale.x = 0.2         
    m.color.a = 1.0
    m.color.r = 0.0
    m.color.g = 0.0
    m.color.b = 1.0



    pub.publish(p);

def gateway():

    rospy.init_node('gateway_pose_stamped_to_marker', anonymous=True)
    rospy.Subscriber("/pose_tag", PoseStamped, callback)

    rospy.spin()
        
if __name__ == '__main__':
    gateway()
