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
	
		# going to sofa offering drink
		tv = rospy.get_param("/script_server/base/tv")
		tv[0] = tv[0] - 0.6 
		tv[1] = tv[1] - 0.0
		tv[2] = 1*3.1415926/4

		handle_base = self.sss.move("base",tv,False)
		if not self.sss.parse:
			starting_time = rospy.Time.now()
			while (not (handle_base.get_state() == 3)) and (30 > (rospy.Time.now().secs-starting_time.secs) ):
				print rospy.Time.now().secs-starting_time.secs
				diff = rospy.Time.now().secs-starting_time.secs
				try:
					(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
					print trans
				except (tf.LookupException, tf.ConnectivityException):
					continue
				self.sss.set_light("red")
				self.sss.sleep(.5)
				self.sss.set_light([0,0,0])
				self.sss.sleep(.5)
		
		self.sss.set_light("green")
		
		self.sss.move("torso","nod")
		self.sss.say(["Hello there"])
		self.sss.say(["can I get you anything to drink"], False)
		self.sss.move("torso","nod")
		if not self.sss.parse:
			print "Please have your choice"
		choice = self.sss.wait_for_input()

		handle_base = self.sss.move("base","park",False)
		self.sss.move("tray","down",False)
		self.blink(handle_base,"red")

		self.sss.sleep(1)
		self.sss.set_light([0,0,0])


	def blink(self, handle, color):
		if not self.sss.parse:
			while not (handle.get_state() == 3):
				self.sss.set_light(color)
				self.sss.sleep(.5)
				self.sss.set_light([0,0,0])
				self.sss.sleep(.5)
			handle.wait()
	 

if __name__ == "__main__":
	SCRIPT = FetchAndCarry()
	SCRIPT.Start()
