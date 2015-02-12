#!/usr/bin/env python
# Convert PoseWithCovarianceStamped to PoseStamped and Path for visualization

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

pub = rospy.Publisher('/output', PoseStamped, queue_size = 10)


def callback(data):
    p = PoseStamped()
    p.header = data.header
    p.header.frame_id = 'world'
    p.pose = data.pose.pose
    
    # Publish PoseStamped 
    pub.publish(p);


def gateway():
    rospy.init_node('gateway_pose_with_covariance_stamped', anonymous = True)
    rospy.Subscriber("/input", PoseWithCovarianceStamped, callback)
    rospy.spin()
        

if __name__ == '__main__':
    gateway()
