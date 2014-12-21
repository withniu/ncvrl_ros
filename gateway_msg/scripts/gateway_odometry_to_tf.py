#!/usr/bin/env python  
import roslib
#roslib.load_manifest('gate')
import rospy

import tf
from nav_msgs.msg import Odometry

def handle_quad_odom(msg):
    br = tf.TransformBroadcaster()
    position = msg.pose.pose.position;
    orientation = msg.pose.pose.orientation;
    br.sendTransform((position.x, position.y, position.z),
                     (orientation.x, orientation.y, orientation.z, orientation.w),
                     rospy.Time.now(),
                     "quad",
                     "world")

if __name__ == '__main__':
    rospy.init_node('quad_tf_broadcaster')
    rospy.Subscriber('/odom',
                     Odometry,
                     handle_quad_odom)
    rospy.spin()

