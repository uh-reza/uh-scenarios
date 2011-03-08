#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script

#import tf
#from geometry_msgs.msg import *

class StaticDemo(script):
		
	def Initialize(self):
		self.sss.set_light("red")

	def Body(self):
		# move to initial positions
		if not self.sss.parse:
			print "Enter to Continue..."
		self.sss.wait_for_input()
	
		self.sss.set_light("red")
		self.sss.say(["Hello Robot House"], False)
		self.sss.sleep(5)
		self.sss.set_light([0,0,0])

	def Run(self):
		self.Body()
		self.Body()
		self.Body()


if __name__ == "__main__":
	SCRIPT = StaticDemo()
	SCRIPT.Start()
