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
		self.sss.sleep(1)
		self.sss.say(["Hello"])
		if not self.sss.parse:
			print "Press enter ...."
		self.sss.wait_for_input()

		self.sss.set_light("yellow")
		self.sss.sleep(1)
		self.sss.say(["Heer is your drink"])
		if not self.sss.parse:
			print "Press enter ...."
		self.sss.wait_for_input()

		self.sss.set_light("red")
		self.sss.sleep(1)
		self.sss.say(["heer is your cloth"])
		if not self.sss.parse:
			print "Press enter ...."
		self.sss.wait_for_input()

		self.sss.set_light("blue")
		self.sss.sleep(1)
		self.sss.say(["Hello"])
		if not self.sss.parse:
			print "Press enter ...."
		self.sss.wait_for_input()

		self.sss.set_light("white")
		self.sss.sleep(1)
		self.sss.say(["Hello"])
		if not self.sss.parse:
			print "Press enter ...."
		self.sss.wait_for_input()

		self.sss.say(["Thank you"])
		self.sss.set_light([0,0,0])
	 

if __name__ == "__main__":
	SCRIPT = FetchAndCarry()
	SCRIPT.Start()
