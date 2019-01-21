from controller import Supervisor
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import math
import numpy as np
import pickle
import matplotlib.pyplot as plt
import os, sys, random, time
from shutil import copyfile
from distutils.dir_util import copy_tree
from copy import deepcopy


class God:
    generations_folder = "../../data/generations/"
    individual_prename = "ind_"
    individual_file_format = ".motion"
    max_generation = 1000
    # initial_motionfile = "../../motions/initial_policy_gradient.motion"
    current_policy = []
    n_popolation = 100
    num_perturbations = n_popolation
    epsilon = 0.1
    initial_filepath = "../../motions/Shoot_copy_corrected.motion"
    fitness_filepath = generations_folder + "fitness.txt"
    # initial_filepath = "../../motions/handCraftedKick.motion"
    n_generation = 1

    column_to_evolve = [6, 7, 8, 9, 10]
    evolve_row_from_to = [70, -20]  # start, at least 1, since the first row is the title in the motion file


    def plot_it(self):
        plt.plot(self.max_scores)
        plt.ylabel('fitness')
        plt.xlabel('generations')
        plt.show()
        # plt.show(block=False)


    def __init__(self, godFile):
        print(": I am the Alpha and the Omega ( Initialized God.py )")
        self.godFile = godFile
        self.motion_util = motion_util()
        self.original_motion_file = self.open_motion_file(self.initial_filepath)
        print("adamo: " + self.initial_filepath)
        self.popolate()
        self.reset_all_score()
        self.current_individual = 0
        self.n_generation = 1
        self.max_scores = [] # each generation


    # First initialization of the popolation
    def popolate(self):
        print(": Let the waters bring forth abundantly the moving creature that hath life ( initialization of the popolation )")
        initial_policy = self.get_policy()
        self.current_policy = initial_policy
        self.create_perturbation_motions(initial_policy)

    def get_policy(self):
        to = self.get_to_line_evolve()

        parameters = []
        for i_row in range(self.evolve_row_from_to[0], to):
            for col in self.column_to_evolve:
                parameters.append(float(self.original_motion_file[i_row][col]))

        self.num_parameters = len(parameters)
        # self.num_parameters = len(self.column_to_evolve) * (to - self.evolve_row_from_to[1] + 1)
        
        return parameters

    def get_to_line_evolve(self):
        if self.evolve_row_from_to[1] < 0:
            to = len(self.original_motion_file) + self.evolve_row_from_to[1]
        elif self.evolve_row_from_to[1] == 0:
            to = len(self.original_motion_file)
        else:
            to = self.evolve_row_from_to[1]
        return to

    def create_perturbation_motions(self, policy):
        next_perturbations = self.get_random_perturbations(policy)
        for i in range(self.n_popolation):
            self.create_individual(next_perturbations[i,:], i)

    def get_random_perturbations(self, policy):
        new_perturbations = np.tile(policy, (self.num_perturbations, 1))
        self.epsilons = (2 * self.epsilon) * np.random.random(new_perturbations.shape) - self.epsilon
        return new_perturbations + self.epsilons

    def wake_up(self):
        # print(": Sacrifice your first born in my name!")
        self.current_individual += 1

        # I don't want to retest the ones already tested:
        while self.current_individual <= self.n_popolation and self.scores[self.current_individual - 1] != 0:
            self.current_individual += 1

        # Have we finished to evaluate the current popolation?
        if self.current_individual > self.n_popolation:
            print(": Noah! Prepare the ark, I have to pee... ( I evaluated all the popolation ) ")

            if self.n_generation >= self.max_generation:
                self.backup()
                return False

            # now repopulate
            self.repopolate()
        return True
            
        # kill the individual not selected, and repopolate
    def repopolate(self):
        # self.plot_it()
        gradient = np.zeros((self.num_parameters))
        gradient = self.calc_gradient(gradient)
        normalized_gradient = gradient/np.linalg.norm(gradient) * self.epsilon
        updated_policy = self.current_policy + normalized_gradient
        self.current_policy = updated_policy
        self.n_generation += 1
        self.create_perturbation_motions(updated_policy)
        self.next_gen()

    def calc_gradient(self, gradient):
        for i in range(self.num_parameters):
            plus = self.get_avg_plus(i)
            zero = self.get_avg_zero(i)
            minus = self.get_avg_minus(i)
            if ( plus > zero or minus > zero ):
                gradient[i] = plus - minus
        return gradient
                
    def get_avg_plus(self, i):
        plus_group = []
        for j in range(self.num_perturbations):
            if self.epsilons[j,i] > 1/3*self.epsilon:
               plus_group.append(self.scores[j])
        avg_plus = sum(plus_group)/len(plus_group)
        return avg_plus                
                    
    def get_avg_zero(self, i):
        zero_group = []
        for j in range(self.num_perturbations):
            if self.epsilons[j,i] > -1/3*self.epsilon and self.epsilons[j,i] < 1/3*self.epsilon:
               zero_group.append(self.scores[j])
        avg_zero = sum(zero_group)/len(zero_group)
        return avg_zero                
        
    def get_avg_minus(self, i):
        minus_group = []
        for j in range(self.num_perturbations):
            if self.epsilons[j,i] < -1/3*self.epsilon:
               minus_group.append(self.scores[j])
        avg_minus = sum(minus_group)/len(minus_group)
        return avg_minus

    # are you passing to the next generation? then you should reinitialize some variables...
    def next_gen(self):
        self.current_individual = 1

        scores = self.scores # backup
        self.max_scores.append(max(scores))
        
        print("the best one is: " + str(scores.index(max(scores))) + " with a score of: " + str(max(scores)))

        self.scores = [0] * self.n_popolation # reset to zero the score array

    def create_individual(self, parameters, i):
        motion_file = deepcopy(self.original_motion_file)

        to = self.get_to_line_evolve()

        ip = 0
        for i_row in range(self.evolve_row_from_to[0], to):
            for col in self.column_to_evolve:
                # to make it respects the limits
                previous_row = i_row - 1
                if previous_row == 0:
                    previous_value = 0
                    delta_t = -1
                else:
                    previous_value = float(motion_file[previous_row][col])
                    delta_t = self.motion_util.get_seconds(motion_file[previous_row][0]) - self.motion_util.get_seconds(motion_file[i_row][0])

                motion_file[i_row][col] = str(
                    self.motion_util.change_value(previous_value, parameters[ip], delta_t, self.original_motion_file[0][col])
                )
                ip+=1


        new_file_folder = self.generations_folder+self.get_generation_folder()
        if not os.path.exists(new_file_folder):
            os.makedirs(new_file_folder)
        new_file_name = new_file_folder + self.individual_prename + str(i) + self.individual_file_format

        # print("closing: " + new_file_name)
        self.close_motion_file(motion_file, new_file_name)
        
    def get_generation_folder(self):
        return "generation" + str(self.n_generation) + "/"

    def set_evaluation(self, score):
        self.scores[self.current_individual-1] = score

    # get the path to the next motion file to use
    def next_motion_filename(self):
        return self.create_individual_filepath(self.current_individual - 1)

    def create_individual_filepath(self, i):
        return self.generations_folder + self.get_generation_folder() + self.individual_prename + str(i) + self.individual_file_format

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
        else:
            try:
                self.loadGod()
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
            print(": I want to sleep!")
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


        score = goal_score * avg_speed*10 * self.speed_max / fallen_score
        return score

    def die(self):
        message = "Sacrifice yourself in My Name!"
        print(": "+ message)
        self.emitter.send(message.encode('utf-8'))
        sys.exit(0)

class motion_util:
    limits_filepath = "../limits.txt"

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
    def add_initial_time_to_motion_file(self, seconds, ):
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




controller = Driver()
controller.initialization()
# controller.run()

controller.god.plot_it()
# print("Scores:")
# print(controller.god.max_scores)
# plt.show()

# controller.only_run(21)

# controller.run_original()
# controller.run_the_best()