from controller import Supervisor
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import math
import numpy as np
import pickle
import os, sys, random, time
from shutil import copyfile
from distutils.dir_util import copy_tree


class God:
    popolation_folder = "../../data/popolation/"
    individual_prename = "ind_"
    individual_file_format = ".motion"
    max_generation = 10000
    backup_every = 50  # generations
    backup_folder = "../../data/backup/"
    adamo_filepath = "../../motions/Shoot_corrected.motion"

    n_popolation = 100
    n_to_select = 10
    n_to_crossover = 40 # need to be an even number

    # motion file info/settings
    # 11 = LAnkleRoll
    column_to_evolve = [6, 7, 8, 9, 10]
    evolve_row_from_to = [70, -20]  # start, at least 1, since the first row is the title in the motion file

    def __init__(self, godFile):
        print(": I am the Alpha and the Omega ( Initialized the God.py )")
        self.godFile = godFile
        self.motion_util = motion_util()
        self.popolate()
        self.reset_all_score()
        self.current_individual = 0
        self.n_generation = 1

    # are you passing to the next generation? then you should reinitialize some variables...
    def next_gen(self, selected):
        self.n_generation += 1
        self.current_individual = 1

        scores = self.scores  # backup
        self.scores = [0] * self.n_popolation  # reset to zero the score array

        # Now I put the score back to the selected ones, so they will not be retested!
        for sel in selected:
            self.scores[sel] = scores[sel]
        print ("The Generation #" + str(self.n_generation) + " is ready!")

    # select the top individuals
    def selection(self):
        print("The top one is the #" + str(self.scores.index(max(self.scores))) + " scored: " + str(max(self.scores)))
        # the top n_to_save, return they indeces
        return np.argpartition(self.scores, -self.n_to_select)[-self.n_to_select:]

    # kill the individual not selected, and repopolate
    def repopolate(self, selected):
        print(": Have a lot of children and grandchildren. ( repopolating ) ")

        individual_crossed = []
        for _ in range(int(self.n_to_crossover/2)):
            first = random.choice(selected)
            temp_selected = np.delete(selected, np.where(selected == first))
            second = random.choice(temp_selected)

            individual_crossed.extend( self.crossover(self.create_individual_filepath(first), self.create_individual_filepath(second)) )

        for i in range(self.n_popolation):
            # I just skipt the number of the ones selected, so I don't have to rename or think to much about it
            if i in selected:
                continue

            if len(individual_crossed) > 0:
                self.close_motion_file(individual_crossed.pop(), self.popolation_folder + self.individual_prename + str(i) + self.individual_file_format)
            else: # random mutation
                self.create_mutated_individual(self.create_individual_filepath(random.choice(selected)), i)

        self.next_gen(selected)


    # First initialization of the popolation
    def popolate(self):
        print(
            ": Let the waters bring forth abundantly the moving creature that hath life ( initialization of the popolation )")
        for i in range(self.n_popolation):
            self.create_mutated_individual(self.adamo_filepath, i)

    def create_mutated_individual(self, start_filename, i):
        adamo = self.open_motion_file(start_filename)

        probability_to_change = 0.04
        percentuage_to_change = range(5, 10 + 1)  # [from, to] 5% - 10% # need the +1
        min_change = 0.01  # 0.1, not 1% !

        # to evolve only some columns
        if self.evolve_row_from_to[1] < 0:
            to = len(adamo) + self.evolve_row_from_to[1]
        elif self.evolve_row_from_to[1] == 0:
            to = len(adamo)
        else:
            to = self.evolve_row_from_to[1]

        for i_row in range(self.evolve_row_from_to[0], to):
            for col in self.column_to_evolve:
                if random.uniform(0, 1) < probability_to_change:
                    # ok let's change it then...
                    perc_change = random.choice(range(percentuage_to_change[0], percentuage_to_change[1])) / 100
                    change = perc_change * float(adamo[i_row][col])
                    if change < min_change:
                        change = min_change

                    # maybe it is negative..
                    if random.uniform(0, 1) > 0.5:
                        change = -change

                    # return in str, because it is much easier to save if is in string!
                    original = adamo[i_row][col]
                    previous_row = i_row -1
                    if previous_row == 0:
                        previous_value = 0
                        delta_t = -1
                    else:
                        previous_value = float(adamo[previous_row][col])
                        delta_t = self.motion_util.get_seconds(adamo[previous_row][0]) - self.motion_util.get_seconds(adamo[i_row][0])
                    adamo[i_row][col] = str(
                        self.motion_util.change_value(previous_value, float(adamo[i_row][col]) + change, delta_t, adamo[0][col])
                     )

                    # let's propagate the change to the neighbors
                    # just one row
                    if i_row - 1 > 0 and i_row + 1 < len(adamo):
                        # if (adamo[i_row-1][col] < original < adamo[i_row+1][col]) and not (adamo[i_row-1][col] < adamo[i_row][col] < adamo[i_row+1][col]):
                        adamo[i_row - 1][col] = str(float(adamo[i_row - 1][col]) + change / 2)
                        adamo[i_row + 1][col] = str(float(adamo[i_row + 1][col]) + change / 2)

        self.close_motion_file(adamo, self.popolation_folder + self.individual_prename + str(i) + self.individual_file_format)

    # CROSSOVER!!
    def crossover(self, ind1, ind2):
        mot1 = self.open_motion_file(ind1)
        mot2 = self.open_motion_file(ind2)

        # to evolve only some columns
        if self.evolve_row_from_to[1] < 0:
            to = len(mot1) + self.evolve_row_from_to[1]
        elif self.evolve_row_from_to[1] == 0:
            to = len(mot1)
        else:
            to = self.evolve_row_from_to[1]


        # let's mix it up
        if random.uniform(0, 1) < 0.5: # let's randomize completely the mix or let's do as chunks, 0.5
            for column in self.column_to_evolve:
                for row in range(self.evolve_row_from_to[0], to+1):
                    if random.uniform(0, 1) < 0.5:
                        mot1[row][column], mot2[row][column] = mot2[row][column], mot1[row][column]
        else: # change from point
            split_point = random.choice(range(self.evolve_row_from_to[0], to + 1))

            for row in range(split_point, to+1):
                mot1[row], mot2[row] = mot2[row], mot1[row]


        return mot1, mot2

    def delete_not_selected(self, selected):
        print(": The end is now upon you. ( selection of the best ones )")

        # selected contains the indeces of the selected ones
        for i in range(self.n_popolation):
            if i not in selected:
                self.delete_individual(i)

    def delete_individual(self, i):
        filename = self.popolation_folder + self.individual_prename + str(i) + self.individual_file_format
        if os.path.exists(filename):
            os.remove(filename)
        else:
            print("The individual does not exist")

    def set_evaluation(self, score):
        self.scores[self.current_individual - 1] = score

    def wake_up(self):
        # print(": Sacrifice your first born in my name!")
        self.current_individual += 1

        # I don't want to retest the ones already tested:
        while self.current_individual <= self.n_popolation and self.scores[self.current_individual - 1] != 0:
            self.current_individual += 1

        # Have we finished to evaluate the current popolation?
        if self.current_individual > self.n_popolation:
            print(": Noah! Prepare the ark, I have to pee... ( I evaluated all the popolation ) ")

            # backup time?
            if self.n_generation % self.backup_every == 0:
                self.backup()
            # stop time?
            if self.n_generation >= self.max_generation:
                self.backup()
                return False
                # self.die()

            # now I should select and repopulate
            selected = self.selection()
            self.delete_not_selected(selected)
            self.repopolate(selected)
        return True

    # get the path to the next motion file to use
    def next_motion_filename(self):
        return self.create_individual_filepath(self.current_individual - 1)

    def create_individual_filepath(self, i):
        return self.popolation_folder + self.individual_prename + str(i) + self.individual_file_format

    # it opens a motion file, and format in an array rows x cols, remember that the first row is the title
    def open_motion_file(self, filepath):
        # first line is the columns' titles...
        with open(filepath) as f:
            content = f.readlines()

        return [x.strip().split(",") for x in content]

    # you give as input an array and a filepath where to save, and it will create back the motion file!
    def close_motion_file(self, content, filepath):
        content = [','.join(x) for x in content]
        with open(filepath, "w+") as f:
            f.write('\n'.join(content))

    # create a backup in data/backup
    def backup(self):
        timestr = time.strftime("%Y%m%d-%H%M%S")
        backup_dst = self.backup_folder + timestr + "_generation_"+ str(self.n_generation) + " best_" + str(self.scores.index(max(self.scores)))
        backup_dst_pop = backup_dst + "/popolation/"

        if not os.path.exists(self.backup_folder):
            os.mkdir(self.backup_folder)  # create the folder "backup" if doesn't exist....
        if not os.path.exists(backup_dst):
            os.mkdir(backup_dst)
            os.mkdir(backup_dst_pop)
        # copy the god file
        copyfile(self.godFile, backup_dst + "/god.npy")
        # copy the popolation folder
        copy_tree(self.popolation_folder, backup_dst_pop)

    def reset_all_score(self):
        self.scores = [0] * self.n_popolation

class Driver (Supervisor):
    timeStep = 128
    MAX_TIME = 10 # Max seconds for simulate!
    football_goal = Polygon([(4.57151, 0.8), (4.57151, -0.8), (5.04, -0.8), (5.04, 0.8)])
    football_goal_point = Point(4.806, 0)
    start_distance = 0 # between the ball and the center of the football goal
    goal = False
    fallen = False

    # to calculate an average of the speed of the ball..
    speed_sum = 0
    speed_count = 0
    speed_max = 0
    max_y_ball = 0 #to make the ball fly >.<

    godFile = "../../data/god.npy"  # this is the file where save the object god between simulations

    def initialization(self):
        if not os.path.exists(self.godFile):
            self.god = God(self.godFile)
            print("Congratulation, you have create a new God")
        else:
            try:
                self.loadGod()
                print("God has been loaded in the simulation")
            except: # nothing
                print("Failed to load the last god")
                self.die()

        # TODO This is for retrocompatibility, to remove after a while:
        self.god.godFile = self.godFile

        # to send messages
        self.emitter = self.getEmitter('emitter')

        self.nao = self.getFromDef('NAO')
        self.t_nao = self.nao.getField('translation')
        self.ball = self.getFromDef('Ball')
        self.t_ball = self.ball.getField('translation')
        self.keyboard.enable(self.timeStep)
        self.keyboard = self.getKeyboard()

        # Store initial coordinates of ball
        self.old_ball_pos = self.ball.getPosition()  # 0 is x, 2 is z
        # Store initial distance from the door
        self.start_distance = self.distance_ball_goal()  # self.football_goal_point.distance(Point(self.old_ball_pos[0], self.old_ball_pos [2]))

        # Store the initial y
        self.init_y = self.t_nao.getSFVec3f()[1]

    def run_original(self):
        self.only_run(-1)

    def only_run(self, num):
        if num == -1:
            motion_file_evaluated = self.god.adamo_filepath
        else:
            motion_file_evaluated = (self.god.popolation_folder + self.god.individual_prename + str(
                num) + self.god.individual_file_format)
        print(motion_file_evaluated)
        self.emitter.send(motion_file_evaluated.encode('utf-8'))

        # Main loop.
        while self.step(self.timeStep) != -1:
            self.check_ball_speed()

            if not self.goal:
                self.is_goal()

            if self.has_robot_fallen() or self.getTime() > self.MAX_TIME:
                print("-------------------")
                print("Simulation Concluded:")
                print("Scored: " + str(self.evaluate()))
                print("-------------------")

                self.step(self.timeStep) # this is for flush the console output
                break

    def run_the_best(self):
        max_score = max(self.god.scores)
        i_best = self.god.scores.index(max_score)
        print("The best one was " + str(i_best) + " (from 0 to x) with a score of " + str(max_score))
        self.only_run(i_best)

    def run(self):
        if not self.god.wake_up():
            self.die()

        # print("this simulation is the number: "+str(self.god.current_individual))

        # let's ask god what motion do
        motion_file_evaluated = self.god.next_motion_filename().encode('utf-8')

        if not os.path.exists(motion_file_evaluated):
            print (motion_file_evaluated)
            print (
                "Error: There is not the motion file that you want to simulate! : S \n This should not happen! Put in PAUSE!!")
            self.waitForUserInputEvent(self.EVENT_NO_EVENT, 100000)

        print("simulating: " + str(motion_file_evaluated))
        self.emitter.send(motion_file_evaluated)

        # Main loop.
        while self.step(self.timeStep) != -1:
            # Deal with the pressed keyboard key.
            # k = self.keyboard.getKey()
            # if k == ord('N'):

            # print(polygon.contains(point))
            # translationValues = self.t_nao.getSFVec3f()
            # print(translationValues)
            # translationValues = self.t_ball.getSFVec3f()
            # print(translationValues)

            # Perform a simulation step, quit the loop when
            # Webots is about to quit.

            self.check_ball_speed()

            if not self.goal:
                self.is_goal()

            if self.has_robot_fallen() or self.getTime() > self.MAX_TIME:
                print("-------------------")
                print("Simulation Concluded")

                self.god.set_evaluation(self.evaluate())

                self.saveGod()

                self.step(self.timeStep)  # this is for flush the console output
                self.nextSimulation()
                break


    def nextSimulation(self):
        self.simulationReset()

    def saveGod(self):
        with open(self.godFile, 'wb') as filehandler:
            pickle.dump(self.god, filehandler)
        print(self.god.scores)

    def loadGod(self):
        with open(self.godFile, 'rb') as filehandler:
            self.god = pickle.load(filehandler)

        print(self.god.scores)

    # How much the ball moved from the previous time_step
    def check_ball_speed(self):
        pos = self.ball.getPosition()
        diffX = math.fabs(pos[0] - self.old_ball_pos[0])
        diffZ = math.fabs(pos[2] - self.old_ball_pos[2])

        try:
            difference = math.sqrt(math.fabs(math.pow(diffX, 2) - math.pow(diffZ, 2)))
        except:
            print("Error! (diffX: " + str(diffX) + " - diffZ: " + str(diffZ) + ")")
            difference = 0
        # print("Ball speed: " + str(difference))

        # Assign new position
        self.old_ball_pos = pos

        # check the average speed only if the ball hasn't scored yet
        if not self.goal:
            if self.t_ball.getSFVec3f()[1] > self.max_y_ball:
                self.max_y_ball = self.t_ball.getSFVec3f()[1]
            self.speed_sum += difference
            self.speed_count += 1
            if difference > self.speed_max:
                self.speed_max = difference

        return difference

    def is_goal(self):
        _ball = self.t_ball.getSFVec3f()
        pos_ball = Point(_ball[0], _ball[2])

        if self.football_goal.contains(pos_ball):
            self.goal = True
            print("GOOOOOAAAAAL!!!")
            return True
        return False

    def has_robot_fallen(self):
        y = self.t_nao.getSFVec3f()[1]  # just y
        tollerance = 0.1
        if math.fabs(self.init_y - y) > tollerance:
            print("Fallen !!! ")
            self.fallen = True
            return True
        return False

    def distance_ball_goal(self):
        _ball = self.t_ball.getSFVec3f()
        return self.football_goal_point.distance(Point(_ball[0], _ball[2]))

    def evaluate(self):
        if self.goal:
            goal_score = 1
        else:
            goal_score = 1 - (self.distance_ball_goal() / self.start_distance)

        if self.fallen:  # you should check how much time is passed from the start to when it fell!
            fallen_score = 100
        else:
            fallen_score = 1

        avg_speed = self.speed_sum / self.speed_count

        print("Max Speed: "+ str(self.speed_max))

        score =  goal_score * avg_speed * 10 * self.speed_max / fallen_score # (10000*self.max_y_ball + 1) *
        return score

    def die(self):
        self.emitter.send("Sacrifice yourself in My Name!".encode('utf-8'))
        sys.exit(0)

class motion_util:
    limits_filepath = "../limits.txt"

    def __init__(self):
        print("initialized")
        self.load_limits()
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
                right_value = old_value + time_difference * self.limits[part_name_index][5] *0.99 # 0.99 just for approximation errors : S

        return right_value

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

    def save(self):
        self.close_motion_file(self.motion, self.filepath)
    def saveCopy(self):
        folder = os.path.dirname(self.filepath)
        filename = os.path.basename(self.filepath)

        name, ext = os.path.splitext(filename)
        new_filepath = folder+"/"+name+"_copy"+ext

        print(new_filepath)
        self.close_motion_file(self.motion, new_filepath)

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
        motion = self.motion
        n_cols = len(motion[0])
        for ic in range(n_cols):
            if motion[0][ic] in self.part_names_limits: # then this part as some limits
                part_name_index = self.part_names_limits.index(motion[0][ic])
                # then check for every row, if it is respecting this limit
                old_value = None
                old_time = None
                remaining = 0
                for ir in range(1, len(motion)):
                    motion[ir][ic] += remaining
                    remaining = 0
                    # make them respect the min and max limits
                    if motion[ir][ic] < self.limits[part_name_index][1]:
                        motion[ir][ic] = self.limits[part_name_index][1]
                    elif motion[ir][ic] > self.limits[part_name_index][3]:
                        motion[ir][ic] = self.limits[part_name_index][3]

                    # now let's check for the time limit!
                    if old_value == None:
                        old_value = motion[ir][ic]
                        old_time = self.get_seconds(motion[ir][0])
                        continue

                    time_difference = old_time - self.get_seconds(motion[ir][0])
                    speed = math.fabs(old_value - motion[ir][ic]) / time_difference
                    if speed > self.limits[part_name_index][5]:
                        # print("too fast!")
                        # let's reduce it
                        new_value = old_value + time_difference * self.limits[part_name_index][5] *0.99 # 0.99 just for approximation errors : S
                        remaining = motion[ir][ic] - new_value
                        motion[ir][ic] = new_value


                    old_value = motion[ir][ic]
                    old_time = self.get_seconds(motion[ir][0])
                if remaining > 0:
                    print("seems is left some remaining...")
        self.motion = motion
        self.close_motion_file(motion, self.filepath+"_")


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
    def add_initial_time_to_motion_file(self, seconds, i_line):
        if i_line == 0: # the first one is the title!!
            i_line = 1
        for il in range(i_line, len(self.motion)):
            s = self.get_seconds(self.motion[il][0])
            s += seconds
            self.motion[il][0] = self.seconds2str(s)


#
# util = motion_util()
# util.open(r"../../motions/Shoot.motion")
# # util.respect_limits()
# util.add_initial_time_to_motion_file(1,1)
# util.saveCopy()


controller = Driver()
controller.initialization()
controller.run()
# controller.only_run(35)

# controller.run_original()
# controller.run_the_best()

# controller.god.backup()