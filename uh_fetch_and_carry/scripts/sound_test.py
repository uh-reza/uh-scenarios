#!/usr/bin/python

import time

import roslib
roslib.load_manifest('uh_fetch_and_carry')
import rospy

from simple_script_server import script

import tf
from geometry_msgs.msg import *

class FetchAndCarry(script):
		
	#def Initialize(self):
			
		#self.sss.set_light("green")

	def Run(self):
		self.sss.set_light("green")
		self.sss.say(["Hello there, can I get you, anything to drink"])
		self.sss.sleep(2)

		self.sss.say(["Here s your drink"])
		self.sss.sleep(.5)
		self.sss.set_light([0,0,0])
	 

if __name__ == "__main__":
	SCRIPT = FetchAndCarry()
	SCRIPT.Start()
