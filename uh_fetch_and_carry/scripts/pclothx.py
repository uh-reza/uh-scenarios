#!/usr/bin/python

import time

import roslib
roslib.load_manifest('uh_fetch_and_carry')
import rospy

from simple_script_server import script

import tf
from geometry_msgs.msg import *

import sys  

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
		self.sss.set_default_velocity("arm",0.5)
		
		# move to initial positions
		handle_arm = self.sss.move("arm","folded",False)
		handle_torso = self.sss.move("torso","home",False)
		self.sss.init("sdh")
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

	def Run(self): 
		listener = tf.TransformListener(True, rospy.Duration(10.0))

		# express behaviour  
		self.sss.move("torso","nod",False)
		self.sss.set_light("white")
		self.sss.sleep(.5)

		# call to localize the base
		if not self.sss.parse:
			print " Localize the robot then enter (1) to continue or (0) to exit"
		choice = self.sss.wait_for_input()

		if not self.sss.parse and choice == "0": 
			self.sss.set_light([0,0,0])
			self.sss.sleep(.5)
			sys.exit("aborting ...")

		# express behaviour  
		self.sss.set_light("yellow")
		self.sss.sleep(.5)

		# move base   
		self.sss.move("base","park")
		park = rospy.get_param("/script_server/base/park")
		self.sss.sleep(.5)
		park[0] = park[0] + 0.45 #0.50 #0.45
		park[1] = park[1] - 0.25 #0.20 #0.25
		handle_base = self.sss.move("base",park,False)
		self.blink(handle_base,"yellow")
		handle_base.wait()

		# express behaviour  
		self.sss.set_light("red")
		self.sss.sleep(.5)

		# move arm
		handle_arm = self.sss.move("arm","pregrasp",False)
		self.blink(handle_arm,"red")
		handle_arm.wait()
		
		self.sss.init("sdh")
		self.sss.move("sdh","cylopen")
		self.sss.sleep(.5)

		handle_arm = self.sss.move("arm","grasp",False)
		self.blink(handle_arm,"red")
		handle_arm.wait()

		# express behaviour  
		self.sss.set_light("red")
		self.sss.sleep(.5)

		self.sss.init("sdh")
		self.sss.move("sdh","cylclosed")
		self.sss.sleep(.5)

		handle_arm = self.sss.move("arm","grasp-to-tray",False)
		self.blink(handle_arm,"red")
		handle_arm.wait()

		# express behaviour  
		self.sss.set_light("yellow")
		self.sss.sleep(.5)

		# move base   
		#handle_base = self.sss.move("base","home",False)
		park[2] = 0
		handle_base = self.sss.move("base",park,False)
		self.blink(handle_base,"yellow")
		handle_base.wait()

		# call to end point
		tv = rospy.get_param("/script_server/base/tv")
		self.sss.sleep(.5)
		while not self.sss.parse and True:
			print " enter HRP point: (1)-P1 (2)-P2 (3)-P3 (4)-P4 (0)-exit"
			choice = self.sss.wait_for_input()

			if choice == "1": 
				tv[0] = tv[0] - 0.0 
				tv[1] = tv[1] - 0.0
				tv[2] = 2*3.1415926/4
				break
			elif choice == "2": 
				tv[0] = tv[0] - 0.0 
				tv[1] = tv[1] - 0.4
				tv[2] = 2*3.1415926/4
				break
			elif choice == "3": 
				tv[0] = tv[0] - 0.6 
				tv[1] = tv[1] - 0.0
				tv[2] = 1*3.1415926/4
				break
			elif choice == "4": 
				tv[0] = tv[0] - 1.0 
				tv[1] = tv[1] + 1.0
				tv[2] = 0*3.1415926/4
				break
			elif choice == "0": 
				self.sss.set_light([0,0,0])
				self.sss.sleep(.5)
				sys.exit("aborting ...")
			elif choice == "5": 
				self.sss.init("sdh")
				self.sss.move("sdh","cylopen")
				self.sss.sleep(.5)
				self.sss.init("sdh")
				self.sss.move("sdh","cylclosed")
				handle_arm = self.sss.move("arm","tray-to-folded",False)
				self.blink(handle_arm,"red")
				handle_arm.wait()
				handle_base = self.sss.move("base","park",False)
				self.blink(handle_base,"yellow")
				handle_base.wait()
				self.sss.set_light([0,0,0])
				self.sss.sleep(.5)
				sys.exit("aborting ...")

		# move base   
		handle_base = self.sss.move("base",tv,False)
		self.blink(handle_base,"yellow")
		handle_base.wait()

		# express behaviour  
		self.sss.move("torso","nod")
		self.sss.set_light("white")
		self.sss.sleep(.5)
		self.sss.say(["Here is your cloth"])

		# call to return to park
		if not self.sss.parse:
			print " Enter to continue ..."
		self.sss.wait_for_input()

		self.sss.init("sdh")
		self.sss.move("sdh","cylopen")
		self.sss.sleep(.5)
		self.sss.init("sdh")
		self.sss.move("sdh","cylclosed")
		self.sss.sleep(.5)

		# express behaviour  
		#self.sss.move("torso","nod")
		self.sss.set_light("yellow")
		self.sss.sleep(.5)

		# move base   
		tv = rospy.get_param("/script_server/base/tv")
		tv[0] = tv[0] - 1.0 
		tv[1] = tv[1] - 0.2
		tv[2] = 2*3.1415926/4
		handle_base = self.sss.move("base",tv,False)
		self.blink(handle_base,"yellow")
		handle_base.wait()

		# express behaviour  
		self.sss.set_light("red")
		self.sss.sleep(.5)

		# move arm 
		handle_arm = self.sss.move("arm","tray-to-folded",False)
		self.blink(handle_arm,"red")
		handle_arm.wait()

		# express behaviour  
		self.sss.set_light("yellow")
		self.sss.sleep(.5)

		# move base   
		tv = rospy.get_param("/script_server/base/tv")
		tv[0] = tv[0] - 1.0 
		tv[1] = tv[1] - 0.2
		tv[2] = 4*3.1415926/4
		handle_base = self.sss.move("base",tv,False)
		self.blink(handle_base,"yellow")
		handle_base.wait()

		# move base   
		handle_base = self.sss.move("base","park",False)
		self.blink(handle_base,"yellow")
		handle_base.wait()

		# express behaviour  
		self.sss.set_light([0,0,0])
		self.sss.sleep(.5)
		#self.sss.move("torso","nod")

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
