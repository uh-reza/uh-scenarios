#!/usr/bin/python

import roslib
roslib.load_manifest('sensor_msgs')
import rospy
from sensor_msgs.msg import *
from sensor_msgs.srv import *

rospy.init_node('set_camera_info_client')
rospy.wait_for_service('/stereo/left/camera/set_camera_info')
set_camera_info = rospy.ServiceProxy('/stereo/left/camera/set_camera_info', SetCameraInfo)
req = SetCameraInfoRequest()
req.camera_info.header.stamp = rospy.Time.now()
req.camera_info.height = 480
req.camera_info.width = 640
#fx = 1046.16197
#fy = 1043.87094
#cx = 695.62128
#cy = 544.32183
#k1 = -0.23268
#k2 = 0.08580
#p1 = 0.00098
#p2 = -0.00022
#req.camera_info.D = [1,2,3,4,5]
#req.camera_info.K = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
#req.camera_info.R = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
#req.camera_info.P = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
req.camera_info.K = [4827.94, 0, 1223.5, 0, 4835.62, 1024.5, 0, 0, 1]
req.camera_info.D = [-0.41527, 0.31874, -0.00197, 0.00071, 0]
req.camera_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
req.camera_info.P = [4827.94, 0, 1223.5, 0, 0, 4835.62, 1024.5, 0, 0, 0, 1, 0]
try:
  resp1 = set_camera_info(req)
except rospy.ServiceException, e:
  print "Service did not process request: %s"%str(e)
