#!/usr/bin/env python  
import roslib
#roslib.load_manifest('gate')
import rospy
import math
import tf2_ros
import tf
from keyboard.msg import Key
# ENU
yaw = 0;
x = 0;
y = 0;
z = 1.0;

dy = 0.2;
dx = 0.2;
dz = 0.1;
dyaw = math.pi / 18;

def handle_cmd(msg):
    global x, y, z, yaw, dx, dy, dz, dyaw
    if msg.code == 97:      # 'a'
        y = y + dy
        if y > 2.0:
            y = 2.0
    elif msg.code == 100:   # 'd'
        y = y - dy
        if y < -2.0:
            y = -2.0
    elif msg.code == 115:   # 's'
        x = x - dx
        if x < -2.0:
            x = -2.0
    elif msg.code == 119:   # 'w'
        x = x + dx
        if x > 2.0:
            x = 2.0
    elif msg.code == 102:   # 'f'
        z = z - dz
        if z < 0.1:
            z = 0.1
    elif msg.code == 114:   # 'r'
        z = z + dz
        if z > 2.0:
            z = 2.0
    elif msg.code == 113:   # 'q'
        yaw = yaw + dyaw
        if yaw > 2 * math.pi:
            yaw = yaw - 2 * math.pi
    elif msg.code == 101:   # 'e'
        yaw = yaw - dyaw
        if yaw < - 2 * math.pi:
            yaw = yaw + 2 * math.pi
    print x, y, z, yaw

def send_tf():    

    br = tf.TransformBroadcaster()
    orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
#    print orientation
    br.sendTransform((x, y, z),
                     (orientation[0], orientation[1], orientation[2], orientation[3]),
                     rospy.Time.now(),
                     "setpoint",
                     "local_origin")

if __name__ == '__main__':
    rospy.init_node('commander_tf_broadcaster')
    rospy.Subscriber('/keyboard/keyup',
                     Key,
                     handle_cmd)
    rate = rospy.Rate(50)   # 50Hz
    while not rospy.is_shutdown():
        send_tf();
        rate.sleep()

