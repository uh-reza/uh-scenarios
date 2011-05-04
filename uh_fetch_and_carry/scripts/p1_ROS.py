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

		self.sss.set_light("white")
		self.sss.sleep(.5)
		# call for help to localize the base
		if not self.sss.parse:
			print "Please localize the robot with rviz"
		self.sss.wait_for_input()
		
	def Run(self): 
		self.sss.set_light("yellow")
		self.sss.sleep(.5)
		listener = tf.TransformListener(True, rospy.Duration(10.0))

		handle_base = self.sss.move("base","park")

		park = rospy.get_param("/script_server/base/park")
		park[0] = park[0] + 0.45
		park[1] = park[1] - 0.25
		handle_base = self.sss.move("base",park,False)
		self.blink(handle_base,"yellow")
		handle_base.wait()

		self.sss.set_light("red")
		self.sss.sleep(.5)
		handle_arm = self.sss.move("arm","pregrasp",False)
		self.blink(handle_arm,"red")
		handle_arm.wait()
		
		self.sss.move("sdh","cylopen")
		self.sss.sleep(1)

		handle_arm = self.sss.move("arm","grasp",False)
		self.blink(handle_arm,"red")
		handle_arm.wait()

		self.sss.set_light("red")
		self.sss.sleep(.5)
		self.sss.move("sdh","cylclosed")

		self.sss.sleep(1)
		handle01 = self.sss.move("arm","grasp-to-tray",False)
		self.blink(handle01,"red")
		handle01.wait()

		self.sss.set_light("yellow")
		self.sss.sleep(.5)
		#park[2] = 0*3.1415926/4
		#handle_base = self.sss.move("base",park,False)
		#self.blink(handle_base,"yellow")
		#handle_base.wait()

		# deliver cup to order position
		# P1
		tv = rospy.get_param("/script_server/base/tv")
		tv[0] = tv[0] - 0.0 
		tv[1] = tv[1] - 0.0
		tv[2] = 2*3.1415926/4
		handle_base = self.sss.move("base",tv,False)
		self.blink(handle_base,"yellow")
		handle_base.wait()

		self.sss.move("torso","nod",False)
		self.sss.set_light("white")
		self.sss.sleep(.5)
		self.sss.say(["Here s your cloth"])

		if not self.sss.parse:
			print "Press enter to return to station"
		self.sss.wait_for_input()

		self.sss.move("sdh","cylopen")
		self.sss.sleep(1)
		self.sss.move("sdh","cylclosed")

		self.sss.set_light("yellow")
		self.sss.sleep(1)

		tv = rospy.get_param("/script_server/base/tv")
		tv[0] = tv[0] - 0.9 
		tv[1] = tv[1] - 0.0
		tv[2] = 0*3.1415926/4
		handle_base = self.sss.move("base",tv,False)
		self.blink(handle_base,"yellow")
		handle_base.wait()

		self.sss.set_light("red")
		self.sss.sleep(.5)
		handle01 = self.sss.move("arm","tray-to-folded",False)
		self.blink(handle01,"red")
		handle01.wait()

		self.sss.set_light("yellow")
		self.sss.sleep(.5)
		handle_base = self.sss.move("base","park",False)
		self.blink(handle_base,"yellow")
		handle_base.wait()

		self.sss.sleep(.5)
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
