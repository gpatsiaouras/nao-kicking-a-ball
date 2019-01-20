from controller import Supervisor
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import math
import numpy as np
import pickle
import matplotlib.pyplot as plt
import os, sys, random

class God:
    generations_folder = "../../data/generations/"
    individual_prename = "ind_"
    individual_file_format = ".motion"
    max_generation = 100 # TODO: not used yet
    initial_motionfile = "../../motions/initial_policy_gradient.motion"
    num_parameters = 8
    current_policy = []
    n_popolation = 100
    num_perturbations = 100
    epsilon = 0.1
    initial_filepath = "../../motions/handCraftedKick.motion"
    n_generation = 0

    def __init__(self):
        self.popolate()
        self.scores = [0] * self.n_popolation
        self.current_individual = 0
        self.n_generation = 0

    # First initialization of the popolation
    def popolate(self):
        print("God: Let the waters bring forth abundantly the moving creature that hath life")
        initial_policy = self.get_policy(self.initial_filepath)
        self.current_policy = initial_policy
        self.create_perturbation_motions(initial_policy)

    def get_policy(self, filepath):
        motion_file = self.open_motion_file(filepath)
        parameters = [0,0,0,0,0,0,0,0]
        # 1. param: pull left knee up
        parameters[0] = float(motion_file[21][11])
        # 2. param: pull left hip up
        parameters[1] = float(motion_file[22][10])
        # 3. param: pull left foot up
        parameters[2] = float(motion_file[23][12])
        # 4. param: stretch right hip
        parameters[3] = float(motion_file[24][16])
        # 5. param: stretch right knee
        parameters[4] = float(motion_file[24][17])
        # 6. param: strech left knee
        parameters[5] = float(motion_file[25][11])
        # 7. param: strech left hip
        parameters[6] = float(motion_file[25][10])
        # 8. param: strech left foot
        parameters[7] = float(motion_file[25][12])

        return parameters

    def create_perturbation_motions(self, policy):
        next_perturbations = self.get_random_perturbations(policy)
        for i in range(self.n_popolation):
            self.create_individual(next_perturbations[i,:], i)

    def get_random_perturbations(self, policy):
        new_perturbations = np.tile(policy, (self.num_perturbations, 1))
        self.epsilons = (2 * self.epsilon) * np.random.random(new_perturbations.shape) - self.epsilon
        return new_perturbations + self.epsilons

    def wake_up(self):
        # print("God: Sacrifice your first born in my name!")
        self.current_individual += 1

        # I don't want to retest the ones already tested:
        while self.current_individual <= self.n_popolation and self.scores[self.current_individual-1] != 0:
            self.current_individual += 1

        # Have we finished to evaluate the current popolation?
        if self.current_individual > self.n_popolation:
            print("God: Noah! Prepare the ark!")

            # now repopulate
            self.repopolate()

        # kill the individual not selected, and repopolate
    def repopolate(self):
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
        if self.n_generation > 100:
            fitness_file = self.open_motion_file(self.generations_folder + "fitness.txt")
            plt.plot([float(item[0]) for item in fitness_file], [float(item[1]) for item in fitness_file])
            plt.ylabel('fitness')
            plt.xlabel('generations')
            plt.show()
        self.current_individual = 1

        scores = self.scores # backup
        fitness_file = self.open_motion_file(self.generations_folder + "fitness.txt")
        fitness_file.append([str(self.n_generation), str(max(scores))])
        self.close_motion_file(fitness_file, self.generations_folder + "fitness.txt")
        self.scores = [0] * self.n_popolation # reset to zero the score array


    def create_individual(self, parameters, i):
        motion_file = self.open_motion_file(self.initial_filepath)

        # 1. param: pull left knee up
        motion_file[21][11] = str(parameters[0])
        # 2. param: pull left hip up
        motion_file[22][10] = str(parameters[1])
        # 3. param: pull left foot up
        motion_file[23][12] = str(parameters[2])
        # 4. param: stretch right hip
        motion_file[24][16] = str(parameters[3])
        # 5. param: stretch right knee
        motion_file[24][17] = str(parameters[4])
        # 6. param: strech left knee
        motion_file[25][11] = str(parameters[5])
        # 7. param: strech left hip
        motion_file[25][10] = str(parameters[6])
        # 8. param: strech left foot
        motion_file[25][12] = str(parameters[7])
        new_file_folder = self.generations_folder+self.get_generation_folder()
        if not os.path.exists(new_file_folder):
            os.makedirs(new_file_folder)
        new_file_name = new_file_folder + self.individual_prename + str(i) + self.individual_file_format
        self.close_motion_file(motion_file, new_file_name)

    def get_generation_folder(self):
        return "generation" + str(self.n_generation) + "/"

    def set_evaluation(self, score):
        self.scores[self.current_individual-1] = score

    # get the path to the next motion file to use
    def next_motion_filename(self):
        return self.create_individual_filepath(self.current_individual-1)
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


class Driver (Supervisor):
    timeStep = 128
    MAX_TIME = 10 # Max seconds for simulate!
    football_goal = Polygon([(4.57151, 0.8), (4.57151, -0.8), (5.04, -0.8), (5.04, 0.8)])
    football_goal_point = Point(4.806, 0)
    start_distance = 0 # between the ball and the center of the football goal
    goal = False
    fallen = False
    godFile = "../../data/god.npy"

    # to calculate an average of the speed..
    speed_sum = 0
    speed_count = 0
    speed_max = 0


    def initialization(self):

        if not os.path.exists(self.godFile):
            self.god = God()
        else:
            try:
                self.loadGod()
            except: # nothing
                print("Failed to load the last god: ")
                sys.exit(0)

        # to send messages
        self.emitter = self.getEmitter('emitter')

        self.nao = self.getFromDef('NAO')
        self.t_nao = self.nao.getField('translation')
        self.ball = self.getFromDef('Ball')
        self.t_ball = self.ball.getField('translation')
        self.keyboard.enable(self.timeStep)
        self.keyboard = self.getKeyboard()

        # Store initial coordinates of ball
        self.old_ball_pos = self.ball.getPosition() # 0 is x, 2 is z
        # Store initial distance from the door
        self.start_distance = self.distance_ball_goal()# self.football_goal_point.distance(Point(self.old_ball_pos[0], self.old_ball_pos [2]))

        # Store the initial y
        self.init_y = self.t_nao.getSFVec3f()[1]

    def only_run(self, num):
        motion_file_evaluated = (self.god.popolation_folder+self.god.individual_prename+str(num)+self.god.individual_file_format).encode('utf-8')
        print(motion_file_evaluated)
        self.emitter.send(motion_file_evaluated)

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


    def run(self):
        self.god.wake_up()

        # print("this simulation is the number: "+str(self.god.current_individual))

        # let's ask god what motion do
        motion_file_evaluated = self.god.next_motion_filename().encode('utf-8')

        if not os.path.exists(motion_file_evaluated):
            print (motion_file_evaluated)
            print ("Error: There is not the motion file that you want to simulate! : S \n This should not happen! Put in PAUSE!!")
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

                self.step(self.timeStep) # this is for flush the console output
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
            print("Error! (diffX: "+str(diffX)+" - diffZ: "+str(diffZ)+")")
            difference = 0
        # print("Ball speed: " + str(difference))

        # Assign new position
        self.old_ball_pos = pos

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
        # else:
            # print("NO")
        return False

    def has_robot_fallen(self):
        y = self.t_nao.getSFVec3f()[1] # just y
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
            goal_score = 1 - (self.distance_ball_goal() / self.start_distance )

        if self.fallen: # you should check how much time is passed from the start to when it fell!
            fallen_score = 100
        else:
            fallen_score = 1

        # todo the average should be calculated at the moment of goal!
        avg_speed = self.speed_sum / self.speed_count
        score = goal_score * avg_speed*10 * self.speed_max / fallen_score
        return score


controller = Driver()
controller.initialization()
controller.run()
# controller.only_run(99)
