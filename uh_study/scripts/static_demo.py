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
		self.sss.init("arm")
		self.sss.init("sdh")
		self.sss.init("base")
		
		# move to initial positions
		handle01 = self.sss.move("arm","folded",False)
		self.sss.move("torso","home",False)
		self.sss.move("sdh","home",False)
		self.sss.move("tray","down")
		handle01.wait()
		if not self.sss.parse:
			print "Please localize the robot with rviz or just Enter to Continue..."
		self.sss.wait_for_input()
		#self.sss.move("base","home")
	
	def StaticDemo(self):
		self.sss.say(["oh Hi welcome to robot house"],True)
		self.sss.move("sdh","cylclosed",False)
		self.sss.move("tray","up",True)
		self.sss.set_light("red")
		self.sss.sleep(5)		
		self.sss.move("tray","down",True)
		self.sss.set_light("blue")
		self.sss.sleep(5)		
		self.sss.move("tray","up",True)
		self.sss.set_light("yellow")
		self.sss.sleep(5)		
		self.sss.move("tray","down",True)
		self.sss.set_light("red")
		self.sss.sleep(5)		
		self.sss.move("sdh","cylopen",False)
		self.sss.move("torso","home",True)
		self.sss.set_light("blue")
		self.sss.sleep(2)		
		self.sss.move("torso","right",True)
		self.sss.set_light("yellow")
		self.sss.sleep(2)		
		self.sss.move("torso","left",False)
		self.sss.set_light("red")
		self.sss.sleep(2)		
		self.sss.move("torso","front",True)
		self.sss.set_light("blue")
		self.sss.sleep(2)		
		self.sss.move("sdh","cylclosed",False)
		self.sss.move("torso","back",True)
		self.sss.set_light("yellow")
		self.sss.sleep(2)		
		self.sss.move("torso","home",True)
		self.sss.set_light("red")
		self.sss.sleep(2)
		self.sss.set_light([0,0,0])		

	def Run(self): 
		self.StaticDemo()


if __name__ == "__main__":
	SCRIPT = GraspScript()
	SCRIPT.Start()
