#!/usr/bin/env python

PACKAGE='camera_info_manager'
import roslib; roslib.load_manifest(PACKAGE)
import sys
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo

# Default service name for most ROS camera drivers.  Would be nice to
# make this a parameter.
service_name = 'stereo/right/camera/set_camera_info'

def test():
    rospy.wait_for_service(service_name)

    # build theoretical calibration for Unibrain Fire-i camera in
    # 640x480 mode
    cinfo = CameraInfo()
    cinfo.width = 1360
    cinfo.height = 1024
    #cx = (cinfo.width - 1.0)/2.0
    #cy = (cinfo.height - 1.0)/2.0
    #fx = fy = 0.0043                    # Unibrain Fire-i focal length
    #fx = 1046.16197
    #fy = 1043.87094
    #cx = 695.62128
    #cy = 544.32183
    #k1 = -0.23268
    #k2 = 0.08580
    #p1 = 0.00098
    #p2 = -0.00022
    #cinfo.K = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
    #cinfo.R = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    #cinfo.P = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]

    cinfo.K = [430.21554970319971, 0.0, 306.6913434743704, 0.0, 430.53169252696676, 227.22480030078816, 0.0, 0.0, 1.0]
    cinfo.D = [-0.33758562758914146, 0.11161239414304096, -0.00021819272592442094, -3.029195446330518e-05, 0]

    #cinfo.K = [4827.94, 0, 1223.5, 0, 4835.62, 1024.5, 0, 0, 1]
    #cinfo.D = [-0.41527, 0.31874, -0.00197, 0.00071, 0]
    #cinfo.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    #cinfo.P = [4827.94, 0, 1223.5, 0, 0, 4835.62, 1024.5, 0, 0, 0, 1, 0]


    try:
        set_camera_info = rospy.ServiceProxy(service_name, SetCameraInfo)
        response = set_camera_info(cinfo)
        rospy.loginfo("set_camera_info: success=" + str(response.success)
                      + ", status=" + str(response.status_message))
        return response.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass

    rospy.loginfo('set_camera_info test completed')

