#!/bin/python2.7

import argparse
import csv


class Converter:

	def __init__(self):
		self.names = list()
		self.times = list()
		self.keys = list()

	def parse_motion_file(self, motion_file_location, webots_time_type):

		with open(motion_file_location) as motion_file:
			reader = csv.DictReader(motion_file, delimiter=',')
			# First populate the names list with all the name of the fields except WEBOTS_MOTION and V1
			# which are the two first
			self.names = reader.fieldnames[2:]
			# The naoqi requires the time slots for every field
			times_row = list()
			# Temporary store the values of each row because reader can only read once.
			rows = list()
			# Do one scan to all the rows to get the times slots
			for row in reader:
				# Activate this if the time is in type 00:00:000
				if webots_time_type:
					times_row.append(self.get_seconds(row['#WEBOTS_MOTION']))
				else:
					times_row.append(float(row['#WEBOTS_MOTION']))
				rows.append(row)

			# Re scan to fill the keys list
			for field in self.names:
				self.times.append(times_row)
				all_values_for_this_field = list()
				for row in rows:
					all_values_for_this_field.append(float(row[field]))

				self.keys.append(all_values_for_this_field)

	def get_seconds(self, time_str):
		m, s, ms = time_str.split(':')
		return float(m) * 60 + float(s) + float(ms) / 1000

	def get_lists(self):
		return self.names, self.times, self.keys

