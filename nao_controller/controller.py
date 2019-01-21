#!/bin/python2.7

import argparse
from converter import Converter
from naoqi import ALProxy
from getkey import getkey, keys

# Constants
UP = 0
DOWN = 1
SIDELEFT = 2
SIDERIGHT = 3
ROTATELEFT = 4
ROTATERIGHT = 5


class Mover:
	def __init__(self, motion_proxy, posture_proxy):
		self.frequency =0.0 # low speed
		self.motion_proxy = motion_proxy
		self.posture_proxy = posture_proxy
		self.movement_variables = [
			[0.5, 0.0, 0.0], # Forward
			[-0.5, 0.0, 0.0], # Backward
			[0.0, 0.5, 0.0], # Side left
			[0.0, -0.5, 0.0], # Side Right
			[0.0, 0.0, 0.5], # Rotate Left
			[0.0, 0.0, -0.5], # Rotate Right
		]
		self.movement_names = [
			"Forward",
			"Backward",
			"Side left",
			"Side right",
			"Rotate left",
			"Rotate right"
		]
		# Initial Pose for kick
		self.kickPosture = {
			"names" : ['HeadPitch', 'HeadYaw', 'LAnklePitch', 'LAnkleRoll', 'LElbowRoll', 'LElbowYaw', 'LHipPitch', 'LHipRoll', 'LHipYawPitch', 'LKneePitch', 'LShoulderPitch', 'LShoulderRoll', 'RAnklePitch', 'RAnkleRoll', 'RElbowRoll', 'RElbowYaw', 'RHipPitch', 'RHipRoll', 'RHipYawPitch', 'RKneePitch', 'RShoulderPitch', 'RShoulderRoll'],
			"times" : [[0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5]],
			"keys" : [[-0.00176254], [-0.000870955], [-0.568126], [-0.002], [-1.39939], [-1.42456], [-0.66], [0.002], [-0.00255762], [1.23146], [2.07004], [0.391505], [-0.55916], [-0.002], [1.40074], [1.41797], [-0.658], [0.002], [-0.00255762], [1.22538], [2.06746], [-0.379032]]
		}

	def initiate_robot(self):
		print("Waking up robot")
		self.motion_proxy.wakeUp()

		print("Going to posture StandInit")
		self.posture_proxy.goToPosture("StandInit", 0.5)

	def move(self, direction):
		print("Moving " + self.movement_names[direction])
		self.motion_proxy.moveToward(
			self.movement_variables[direction][0],
			self.movement_variables[direction][1],
			self.movement_variables[direction][2],
			[["Frequency", self.frequency]]
		)

	def perform_move_from_motion_file(self, motion_name, motion_file):
		print("Executing " + motion_name)
		converter = Converter()
		converter.parse_motion_file(motion_file, True)
		names, times, keys = converter.get_lists()

		self.get_in_posture_for_kick()
		self.motion_proxy.angleInterpolation(names, keys, times, True)

	def get_in_posture_for_kick(self):
		self.motion_proxy.angleInterpolation(self.kickPosture['names'], self.kickPosture['keys'], self.kickPosture['times'], True)

def print_instructions():
	print("Instructions of use:\
		\n\tArrow Up: Move forward\
		\n\tArrow Down: Move backwards\
		\n\tArrow Left: Move Sideways Left\
		\n\tArrow Right: Move Sideways Right\
		\n\tKey A: Rotate Anticlockwise\
		\n\tKey D: Rotate Clockwise\
		\n\tKey K: Kick\
		\n\tKey H: Display Help\
	")

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("--ip", type=str, default="127.0.0.1", help="Robot ip address")
	parser.add_argument("--port", type=int, default=9559, help="Robot port number")

	args = parser.parse_args()

	motion_proxy = ALProxy("ALMotion", args.ip, args.port)
	posture_proxy = ALProxy("ALRobotPosture", args.ip, args.port)

	mover = Mover(motion_proxy, posture_proxy)
	mover.initiate_robot()

	print_instructions()

	try:
		while True:
			key = getkey()
			if key == keys.UP:
				mover.move(UP)
			elif key == keys.DOWN:
				mover.move(DOWN)
			elif key == keys.LEFT:
				mover.move(SIDELEFT)
			elif key == keys.RIGHT:
				mover.move(SIDERIGHT)
			elif key == 'a':
				mover.move(ROTATELEFT)
			elif key == 'd':
				mover.move(ROTATERIGHT)
			elif key == 's':
				posture_proxy.goToPosture("StandInit", 1)
			elif key == 'h':
				print_instructions()
			elif key == 'k':
				motion_proxy.stopMove()
				mover.perform_move_from_motion_file("Kick", "../motions/Shoot_corrected.motion")
			else:
				print("Move interrupted")
				motion_proxy.stopMove()
	except KeyboardInterrupt:
		print("Quiting...")
		motion_proxy.rest()
		print("Robot in rest position")