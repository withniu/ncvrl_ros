#!/usr/bin/env python
# Convert PoseWithCovarianceStamped to PoseStamped and Path for visualization

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

pub = rospy.Publisher('/output', PoseStamped, queue_size = 10)
pub_path = rospy.Publisher('/path_output', Path, queue_size = 10)

path = Path()

def callback(data):
    p = PoseStamped()
    p.header = data.header
    p.header.frame_id = 'world'
    p.pose = data.pose.pose
    path.poses.append(p)
    path.header = data.header
    
    # Publish PoseStamped 
    pub.publish(p);
    # Publish path every other 100 data points
    if p.header.seq % 100 == 0:
        pub_path.publish(path)


def gateway():
    rospy.init_node('gateway_pose_with_covariance_stamped', anonymous = True)
    rospy.Subscriber("/input", PoseWithCovarianceStamped, callback)
    rospy.spin()
        

if __name__ == '__main__':
    gateway()
