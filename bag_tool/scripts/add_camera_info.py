#!/usr/bin/env python
import rosbag
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

if __name__ == '__main__':
    import sys
    filename = sys.argv[1]

    bag = rosbag.Bag(filename, 'a')
    for topic, msg, t in bag.read_messages(topics=['/mv_26802491/image_raw']):
        print msg.header.frame_id
	msg_camera_info = CameraInfo()
	msg_camera_info.header = msg.header
	msg_camera_info.height = msg.height
	msg_camera_info.width = msg.width
	msg_camera_info.distortion_model = 'plumb_bob'
	msg_camera_info.D = [-0.3158, 0.1439, -1.5443e-04, 5.1411e-04, -0.0397]
	msg_camera_info.K = [366.6719, 0, 353.4861, 0, 367.3712, 247.5548, 0, 0, 1.0]
	msg_camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
	msg_camera_info.P = [366.6719, 0, 353.4861, 0, 0, 367.3712, 247.5548, 0, 0, 0, 1.0, 0]

	bag.write('/mv_26802491/camera_info', msg_camera_info, t)
#	msg_test = String('hehe')	
#	bag.write('/mv_26802491/test', msg_test, t)	
    bag.close()
