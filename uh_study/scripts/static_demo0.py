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
		self.sss.init("sdh")
		self.sss.init("tray")
		self.sss.init("torso")
		self.sss.init("arm")
		#self.sss.init("eyes")
		#self.sss.init("base")
		
		# move to initial positions
		self.sss.set_light("red")
		self.sss.sleep(1)
		self.sss.set_light("blue")
		self.sss.sleep(1)
		self.sss.set_light("yellow")
		self.sss.sleep(1)
		self.sss.set_light([0,0,0])

		handle01 = self.sss.move("arm","folded",False)
		self.sss.move("torso","home",False)
		self.sss.move("sdh","home",False)
		self.sss.move("tray","down",False)
		handle01.wait()
		if not self.sss.parse:
			print "Enter to Continue..."
		self.sss.wait_for_input()
		#self.sss.move("base","home")
	
	def StaticDemo(self):
		self.sss.say(["Hello"], False)
		self.sss.set_light("red")
		self.sss.sleep(3)
		self.sss.say(["Welcome Robot House"], False)
		self.sss.set_light("blue")
		self.sss.sleep(3)
		self.sss.say(["I Am Care O Bot"], False)
		self.sss.set_light("yellow")
		self.sss.sleep(3)
		self.sss.set_light([0,0,0])

		#self.sss.move("torso","front",True)
		#self.sss.move("torso","home",True)
		#self.sss.move("torso","back",True)
		#self.sss.move("torso","home",True)
		#self.sss.move("torso","left",True)
		#self.sss.move("torso","home",True)
		#self.sss.move("torso","right",True)
		#self.sss.move("torso","home",True)

		#self.sss.move("eyes","back",False)
		self.sss.move("arm","pregrasp",True)
		self.sss.set_light("yellow")
		self.sss.move("sdh","cylopen",True)
		self.sss.sleep(1)
		self.sss.move("arm","grasp",True)

		self.sss.set_light("red")
		self.sss.sleep(1)
		self.sss.set_light("blue")
		self.sss.sleep(1)
		self.sss.set_light("yellow")
		self.sss.sleep(1)
		self.sss.set_light([0,0,0])

		self.sss.move("sdh","cylclosed",True)
		self.sss.sleep(1)
		self.sss.move("arm","pregrasp",True)

		#self.sss.move("eyes","front",False)
		self.sss.move("arm","intermediateback",True)
		self.sss.sleep(1)
		handle01 = self.sss.move("arm","intermediatefront",False)
		self.sss.move("tray","up",False)
		handle01.wait()

		self.sss.move("arm","overtablet",True)
		self.sss.sleep(1)
		self.sss.move("arm","tablet",True)
		self.sss.sleep(1)
		self.sss.move("sdh","cylopen",True)

		self.sss.set_light("red")
		self.sss.sleep(1)
		self.sss.set_light("blue")
		self.sss.sleep(1)
		self.sss.set_light("yellow")
		self.sss.sleep(1)
		self.sss.set_light([0,0,0])

		self.sss.move("arm","overtablet",True)
		self.sss.sleep(1)
		self.sss.move("sdh","cylclosed",True)

		self.sss.move("arm","intermediatefront",True)
		self.sss.move("arm","intermediateback",True)
		handle01 = self.sss.move("arm","folded",False)
		handle01.wait()
		if not self.sss.parse:
			print "Enter to Continue..."
		self.sss.wait_for_input()
		self.sss.set_light("red")
		self.sss.sleep(2)
		self.sss.set_light("blue")
		self.sss.sleep(2)
		self.sss.set_light("yellow")
		self.sss.sleep(2)
		self.sss.set_light([0,0,0])
		self.sss.say(["Thank You"], False)
		self.sss.move("tray","down",True)

	def Run(self): 
		self.StaticDemo()


if __name__ == "__main__":
	SCRIPT = GraspScript()
	SCRIPT.Start()
