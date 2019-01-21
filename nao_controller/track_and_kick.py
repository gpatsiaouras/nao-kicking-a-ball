#!/bin/python2.7

import time
import argparse
from naoqi import ALProxy
from controller import Mover

class TrackAndKick():
	def __init__(self, robot_ip, robot_port):
		print "Connecting to", robot_ip, "with port", robot_port
		# Constants
		self.fractionMaxSpeed = 0.8
		self.diameterOfBall = 0.8
		self.targetName = "RedBall"

		self.motion = ALProxy("ALMotion", robot_ip, robot_port)
		self.posture = ALProxy("ALRobotPosture", robot_ip, robot_port)
		self.tracker = ALProxy("ALTracker", robot_ip, robot_port)

	def initiate(self):
		# First, wake up.
		self.motion.wakeUp()

		# Go to posture stand
		self.posture.goToPosture("StandInit", self.fractionMaxSpeed)

		# Add target to track.
		self.tracker.registerTarget(self.targetName, self.diameterOfBall)

		# set mode
		mode = "Move"
		self.tracker.setMode(mode)

		# Then, start tracker.
		self.tracker.track(self.targetName)

	def run(self):
		try:
			while True:
				if self.tracker.isTargetLost():
					print("Lost Target")
				else:
					print self.tracker.getTargetPosition()
					if self.tracker.getTargetPosition()[0] < 7:
						print("Closer than 30cm")
						self.motion.stopMove()
				time.sleep(1)
		except KeyboardInterrupt:
			print
			print "Interrupted by user"
			print "Stopping..."

		# Stop tracker, go to posture Sit.
		self.tracker.stopTracker()
		self.tracker.unregisterAllTargets()
		# self.posture.goToPosture("Sit", self.fractionMaxSpeed)
		# self.motion.rest()

		print "Track and Kick stopped."

if __name__ == "__main__" :

	parser = argparse.ArgumentParser()
	parser.add_argument("--ip", type=str, default="127.0.0.1", help="Robot ip address")
	parser.add_argument("--port", type=int, default=9559, help="Robot port number")

	args = parser.parse_args()

	tak = TrackAndKick(args.ip, args.port)
	tak.initiate()
	tak.run()