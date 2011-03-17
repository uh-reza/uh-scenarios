#!/usr/bin/env python
import roslib; roslib.load_manifest('uh_bringup')
import rospy
from cob_relayboard.msg import EmergencyStopState
global oldstate 
oldstate = 0
def callback(msg):
	global oldstate
	if oldstate != msg.emergency_state:
		if msg.emergency_state != 0:
			rospy.set_param('/script_server/pause', True)
	oldstate = msg.emergency_state

def listener():
    rospy.init_node('emergency_watchdog', anonymous=True)
    rospy.Subscriber("/emergency_stop_state", EmergencyStopState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()


