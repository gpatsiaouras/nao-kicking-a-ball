from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import math
import numpy as np
import pickle
import os, sys, random, time
from shutil import copyfile
from distutils.dir_util import copy_tree
import matplotlib.pyplot as plt
from copy import deepcopy


class motion_util:
    limits_filepath = "../controllers/limits.txt"

    def __init__(self):
        print("initialized")
        self.load_limits()

    def load_limits(self):
        # first line is the columns' titles...
        part_name = []
        with open(self.limits_filepath) as f:
            content = f.readlines()

        # for line in content:
        part_name = [x.strip().split(":")[0] for x in content]
        content = [x.strip().split(":")[1] for x in content]
        content = [x.strip().split(",") for x in content]
        for line in content:
            line[1] = float(line[1])
            line[3] = float(line[3])
            line[5] = float(line[5])
            line[7] = float(line[7])

        self.part_names_limits = part_name
        self.limits = content

    def open(self, filepath):
        self.motion = self.open_motion_file(filepath)
        self.filepath = filepath

    # it opens a motion file, and format in an array rows x cols, remember that the first row is the title
    def open_motion_file(self, filepath):
        # first line is the columns' titles...
        with open(filepath) as f:
            content = f.readlines()

        content = [x.strip().split(",") for x in content]
        for ir in range(1, len(content)):
            for ic in range(2, len(content[0])):
                content[ir][ic] = float(content[ir][ic])
        return content

    def change_value(self, previous_value, new_value, delta_t, column_name):
        if column_name not in self.part_names_limits:
            return new_value

        part_name_index = self.part_names_limits.index(column_name)
        right_value = new_value
        if new_value < self.limits[part_name_index][1]:
            right_value = self.limits[part_name_index][1]
        elif new_value > self.limits[part_name_index][3]:
            right_value = self.limits[part_name_index][3]

        # now let's check for speed
        if delta_t  != -1: # if it is, then it is the first row, and it is ignored the speed constraint!
            speed = math.fabs(previous_value - new_value) / delta_t
            if speed > self.limits[part_name_index][5]:
                right_value = old_value + time_difference * self.limits[part_name_index][5] *0.999 # 0.99 just for approximation errors : S

        return right_value

    def save(self):
        self.close_motion_file(self.motion, self.filepath)
    def saveCopy(self):
        self.close_motion_file(self.motion, self.getCopyName(self.filepath, "_copy"))
    def getCopyName(self, filepath, add_name):
        folder = os.path.dirname(filepath)
        filename = os.path.basename(filepath)

        name, ext = os.path.splitext(filename)
        new_filepath = folder+"/"+name+add_name+ext
        return new_filepath

    # you give as input an array and a filepath where to save, and it will create back the motion file!
    def close_motion_file(self, content, filepath):
        for ir in range(len(content)):
            for ic in range(len(content[ir])):
                content[ir][ic] = str(content[ir][ic])

        content = [','.join(x) for x in content]
        with open(filepath, "w+") as f:
            f.write('\n'.join(content))
        print("saved here "+filepath)

    def respect_limits(self):
        motion = deepcopy(self.motion)
        n_cols = len(motion[0])
        for ic in range(n_cols):
            if motion[0][ic] in self.part_names_limits: # then this part as some limits
                part_name_index = self.part_names_limits.index(motion[0][ic])
                # then check for every row, if it is respecting this limit
                old_value = None
                old_time = None
                # remaining = 0
                for ir in range(1, len(motion)):
                    # motion[ir][ic] += remaining
                    # remaining = 0

                    # now let's check for the time limit!
                    if old_value is not None:
                        time_difference = self.get_seconds(motion[ir][0]) - old_time
                        speed = math.fabs(old_value - motion[ir][ic]) / time_difference
                        if speed > self.limits[part_name_index][5]:
                            # print("too fast!")
                            # let's reduce it
                            sign = 1
                            if motion[ir][ic] - old_value < 0:
                                sign = -1
                            new_value = old_value + sign * time_difference * self.limits[part_name_index][5] *0.999 # 0.99 just for approximation errors : S
                            remaining = motion[ir][ic] - new_value
                            motion[ir][ic] = new_value

                    # make them respect the min and max limits
                    if motion[ir][ic] < self.limits[part_name_index][1]:
                        motion[ir][ic] = self.limits[part_name_index][1]
                        # if remaining < 0:
                        #     remaining = 0
                    elif motion[ir][ic] > self.limits[part_name_index][3]:
                        motion[ir][ic] = self.limits[part_name_index][3]
                        # if remaining > 0:
                        #     remaining = 0

                    old_value = motion[ir][ic]
                    old_time = self.get_seconds(motion[ir][0])
                # if remaining > 0:
                #     print("seems is left some remaining...")

        #Re add the titles
        print(motion[0])

        self.motion = motion
        self.close_motion_file(motion, self.getCopyName(self.filepath, '_corrected'))
    def check_limits(self, verbose):
        motion = deepcopy(self.motion)
        titles = motion.pop(0)
        n_cols = len(motion[0])
        min_error_count = 0
        max_error_count = 0
        speed_error_count = 0
        for ic in range(n_cols):
            if titles[ic] in self.part_names_limits: # then this part as some limits
                part_name_index = self.part_names_limits.index(titles[ic])
                # then check for every row, if it is respecting this limit
                old_value = None
                old_time = None
                for ir in range(len(motion)):
                    # make them respect the min and max limits
                    if motion[ir][ic] < self.limits[part_name_index][1]:
                        if verbose:
                            print(" pose"+str(ir)+" and column "+str(ic)+" broke the min limit")
                        min_error_count += 1
                    elif motion[ir][ic] > self.limits[part_name_index][3]:
                        if verbose:
                            print(" pose"+str(ir)+" and column "+str(ic)+" broke the max limit")
                        max_error_count += 1

                    # now let's check for the time limit!
                    if old_value == None:
                        old_value = motion[ir][ic]
                        old_time = self.get_seconds(motion[ir][0])
                        continue

                    time_difference = self.get_seconds(motion[ir][0]) - old_time
                    speed = math.fabs(motion[ir][ic] - old_value) / time_difference
                    if speed > self.limits[part_name_index][5]:
                        if verbose:
                            print(" pose"+str(ir)+" and column "+str(ic)+" broken the SPEED limit" + "speed: " + str(speed) + " (limit: "+str(self.limits[part_name_index][5]) + ")" )
                        speed_error_count += 1


                    old_value = motion[ir][ic]
                    old_time = self.get_seconds(motion[ir][0])

        if max_error_count + min_error_count + speed_error_count == 0:
            print("The motion file looks ok! : )")
            return True

        print("The motion file present "+str(min_error_count)+" errors of MIN Limit")
        print("The motion file present "+str(max_error_count)+" errors of MAX Limit")
        print("The motion file got "+str(speed_error_count)+" tickets for exceed the Speed Limits")
        return False


    def get_seconds(self, time_str):
        m, s, ms = time_str.split(':')
        return float(m) * 60 + float(s) + float(ms) / 1000
    def seconds2str(self, seconds):
        ms = int(round((seconds % 1) * 1000))
        s = math.floor(seconds) % 60
        m = math.floor(seconds / 60)
        return str(m).zfill(2)+":"+str(s).zfill(2)+":"+str(ms).zfill(3)

    # adds milliseconds from the start and iterate to everyother row
    # to add 40ms call it with seconds = 0.04
    # to add from the first row, call it with i_line = 1
    def add_initial_time_to_motion_file(self, seconds):
        # if i_line == 0: # the first one is the title!!
        i_line = 1
        for il in range(i_line, len(self.motion)):
            s = self.get_seconds(self.motion[il][0])
            s += seconds
            self.motion[il][0] = self.seconds2str(s)

    # can show much more, but let's show just one per time: column_to_show
    def plot_motion(self, column_to_show):
        motion = deepcopy(self.motion)
        titles = motion.pop(0)
        # col_to_skip = [0,1] # 0 = time, 1 = pose#
        data = [] # in row

        for col in range(len(motion[0])):
            # if col in col_to_skip:
            #     continue

            array = [motion[row][col] for row in range(len(motion))]
            data.append(array)

        # lest's extract the seconds
        timesteps = [row[1].split("ose")[1] for row in motion]

        plt.plot(timesteps, data[column_to_show])
        plt.ylabel('rads')
        plt.xlabel('time')
        plt.show()





util = motion_util()
util.open(r"../motions/jonas_interpolated.motion")
util.respect_limits()
util.add_initial_time_to_motion_file(0.1)
util.saveCopy()