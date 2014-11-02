#!/usr/bin/env python
# Publish marker msg for the court lines
# Yedong Niu, 10/2014

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def gateway():

    rospy.init_node('gateway_marker', anonymous=True)
    pub = rospy.Publisher('/gateway/marker', Marker, queue_size = 10)

    m = Marker()
    m.header.frame_id = '/world'
#    m.header.stamp = rospy.get_time()
    m.ns = 'ncvrl'
    m.id = 0
    m.type = 5
    m.pose.position.x = 0
    m.pose.position.y = 0
    m.pose.position.z = 0
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1.0
    m.scale.x = 0.2         # Line width
    m.color.a = 1.0
    m.color.r = 0.0
    m.color.g = 0.0
    m.color.b = 1.0

    # Marker
    w = 15.24               # Width(AF) in meters
    l = 28.6512             # Length(AC) in meters

    # D --- E --- F
    # |           |
    # C --- B --- A
    points = list()
    
    # Line AF
    p = Point(0, 0, 0)
    points.append(p)
    p = Point(w, 0, 0)
    points.append(p)
    # Line FD
    points.append(p)
    p = Point(w, l, 0)
    points.append(p)
    # Line DC
    points.append(p)
    p = Point(0, l, 0)
    points.append(p)
    # Line CA
    points.append(p)
    p = Point(0, 0, 0)
    points.append(p)
    # Line BE
    p = Point(0, l/2, 0)
    points.append(p)
    p = Point(w, l/2, 0)
    points.append(p)
    
    m.points = points
    
    # Publish at 1Hz
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
      pub.publish(m)
      r.sleep()
  
if __name__ == '__main__':
    gateway()
