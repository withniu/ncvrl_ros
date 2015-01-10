#!/usr/bin/env python  
import roslib
#roslib.load_manifest('gate')
import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def handle_pose_sfm(msg):
    br = tf.TransformBroadcaster()
    position = msg.pose.position;
    orientation = msg.pose.orientation;
    br.sendTransform((position.x, position.y, position.z),
                     (orientation.x, orientation.y, orientation.z, orientation.w),
                     rospy.Time.now(),
                     "cam_sfm",
                     "world")

if __name__ == '__main__':
    rospy.init_node('sfm_tf_broadcaster')
    rospy.Subscriber('/pose_sfm',
                     PoseStamped,
                     handle_pose_sfm)
    rospy.spin()

