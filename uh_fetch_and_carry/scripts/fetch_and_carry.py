#!/usr/bin/python

import time

import roslib
roslib.load_manifest('uh_fetch_and_carry')
import rospy

from simple_script_server import script

import tf
from geometry_msgs.msg import *

class FetchAndCarry(script):
		
	def Initialize(self):
		# initialize components
		self.sss.init("tray")
		self.sss.init("torso")
		self.sss.init("arm")
		self.sss.init("sdh")
		self.sss.init("base")

		# revocer components in case there was an emergency stop before
		self.sss.recover("tray")
		self.sss.recover("torso")
		self.sss.recover("arm")
		self.sss.recover("base")

		# set default configurations
		self.sss.set_default_velocity("arm",0.3)
		
		# move to initial positions
		handle_arm = self.sss.move("arm","folded",False)
		handle_torso = self.sss.move("torso","home",False)
		handle_sdh = self.sss.move("sdh","home",False)
		handle_tray = self.sss.move("tray","down")
		handle_arm.wait()
		handle_torso.wait()
		handle_sdh.wait()

		# check, if all components are working
		retval = handle_arm.get_error_code()
		if retval > 0:
			rospy.logerr("error in arm, aborting...")
			return False

		retval = handle_torso.get_error_code()
		if retval > 0:
			rospy.logerr("error in torso, aborting...")
			return False

		retval = handle_sdh.get_error_code()
		if retval > 0:
			rospy.logerr("error in sdh, aborting...")
			return False

		retval = handle_tray.get_error_code()
		if retval > 0:
			rospy.logerr("error in tray, aborting...")
			return False

		# call for help to localize the base
		if not self.sss.parse:
			print "Please localize the robot with rviz"
		self.sss.wait_for_input()
		
	def Run(self): 
		listener = tf.TransformListener(True, rospy.Duration(10.0))
	
		# prepare for grasping
		self.sss.move("base","shelf")
		self.sss.move("arm","pregrasp")
		handle_sdh = self.sss.move("sdh","cylopen",False)

		#self.sss.wait_for_input()

		# caculate tranformations, we need cup coordinates in arm_7_link coordinate system
		cup = PointStamped()
		cup.header.stamp = rospy.Time.now()
		cup.header.frame_id = "/map"
		cup.point.x = -1.6
		cup.point.y = 1.0
		cup.point.z = 0.86
		self.sss.sleep(2) # wait for transform to be calculated
		handle_sdh.wait()
		
		if not self.sss.parse:
			cup = listener.transformPoint('/arm_7_link',cup)
			# transform grasp point to sdh center
			cup.point.z = cup.point.z + 0.2

		# move in front of cup
		pregrasp_distance = 0.2
		self.sss.move_cart_rel("arm",[[cup.point.x, cup.point.y, cup.point.z+pregrasp_distance], [0, 0, 0]])

		# move to cup
		self.sss.move_cart_rel("arm",[[0.0, 0.0, -pregrasp_distance], [0, 0, 0]])
		# grasp cup
		self.sss.move("sdh","cylclosed")
		# lift cup
		self.sss.move_cart_rel("arm",[[0.0, 0.1, 0.0], [0, 0, 0]])
	

		# place cup on tray
		handle01 = self.sss.move("arm","grasp-to-tray",False)
		self.sss.move("tray","up")
		handle01.wait()
		self.sss.move("sdh","cylopen")
		self.sss.move_cart_rel("arm",[[0.0, 0.0, 0.2], [0, 0, 0]])
		handle01 = self.sss.move("arm","tray-to-folded",False)
		self.sss.sleep(4)
		self.sss.move("sdh","cylclosed",False)
		handle01.wait()

		# deliver cup to order position
		self.sss.move("base","order")
		self.sss.say("Here's your drink.")
		self.sss.move("torso","nod")

if __name__ == "__main__":
	SCRIPT = FetchAndCarry()
	SCRIPT.Start()
