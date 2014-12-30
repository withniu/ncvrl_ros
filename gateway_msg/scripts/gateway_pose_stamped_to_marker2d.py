#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


pub = rospy.Publisher('/gateway/marker2d', Marker, queue_size = 10)

def callback(data):

    global listener    
    try:
        p = listener.transformPose('world', data)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return


    m = Marker()
    m.header = data.header
    m.header.frame_id = 'world'
#    m.header.stamp = rospy.get_time()
    m.ns = 'ncvrl'
    m.id = 0
    m.type = 3
    m.pose.position.x = p.pose.position.x
    m.pose.position.y = p.pose.position.y
    m.pose.position.z = 0
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1.0


    m.scale.x = 0.5         
    m.scale.y = 0.5         
    m.scale.z = 0.01         
    m.color.a = 0.2
    m.color.r = 0.0
    m.color.g = 1.0
    m.color.b = 0.0

    m.frame_locked = 1

    pub.publish(m);

def gateway():

    rospy.init_node('gateway_pose_stamped_to_marker2d', anonymous=True)
    rospy.Subscriber("/pose_tag", PoseStamped, callback)
    global listener
    listener = tf.TransformListener()
    
    rospy.spin()
        
if __name__ == '__main__':
    gateway()
