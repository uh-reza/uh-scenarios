#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script

import tf
from geometry_msgs.msg import *

class GraspScript(script):
		
	def Initialize(self):
		# initialize components (not needed for simulation)
		self.sss.init("tray")
		self.sss.init("torso")
		self.sss.init("base")
		self.sss.init("arm")
		self.sss.init("sdh")
		self.sss.say(["Hello"], False)

		if not self.sss.parse:
			print "Please localize the robot with rviz"
		self.sss.wait_for_input()

		self.sss.move("base", "home1")
		self.sss.set_light("red")
		self.sss.sleep(1)		
		self.sss.say(["home"], False)
		self.sss.set_light("blue")
		self.sss.sleep(1)
		self.sss.set_light("yellow")
		self.sss.sleep(1)
		self.sss.set_light([0,0,0])

		self.sss.move("base", "kitchen3")
		self.sss.set_light("red")
		self.sss.sleep(1)		
		self.sss.say(["kitchen"], False)
		self.sss.set_light("blue")
		self.sss.sleep(1)
		self.sss.set_light("yellow")
		self.sss.sleep(1)
		self.sss.set_light([0,0,0])

		self.sss.move("base", "home3")

		self.sss.move("base", "stair4")
		self.sss.set_light("red")
		self.sss.sleep(1)		
		self.sss.say(["stair"], False)
		self.sss.set_light("blue")
		self.sss.sleep(1)
		self.sss.set_light("yellow")
		self.sss.sleep(1)
		self.sss.set_light([0,0,0])

		self.sss.move("base", "table4")
		self.sss.set_light("red")
		self.sss.sleep(1)		
		self.sss.say(["table"], False)
		self.sss.set_light("blue")
		self.sss.sleep(1)
		self.sss.set_light("yellow")
		self.sss.sleep(1)
		self.sss.set_light([0,0,0])

		self.sss.move("base", "sofa2")
		self.sss.set_light("red")
		self.sss.sleep(1)		
		self.sss.say(["sofa"], False)
		self.sss.set_light("blue")
		self.sss.sleep(1)
		self.sss.set_light("yellow")
		self.sss.sleep(1)
		self.sss.set_light([0,0,0])

		self.sss.move("base", "tv4")
		self.sss.set_light("red")
		self.sss.sleep(1)		
		self.sss.say(["tv"], False)
		self.sss.set_light("blue")
		self.sss.sleep(1)
		self.sss.set_light("yellow")
		self.sss.sleep(1)
		self.sss.set_light([0,0,0])

		self.sss.move("base", "table4")
		self.sss.set_light("red")
		self.sss.sleep(1)		
		self.sss.set_light([0,0,0])

		self.sss.move("base", "stair4")
		self.sss.set_light("blue")
		self.sss.sleep(1)
		self.sss.set_light([0,0,0])

		self.sss.move("base", "home1")
		self.sss.set_light("yellow")
		self.sss.sleep(1)
		self.sss.set_light([0,0,0])

		self.sss.move("base", "park2")
		self.sss.say(["park"], False)


if __name__ == "__main__":
	SCRIPT = GraspScript()
	SCRIPT.Start()
