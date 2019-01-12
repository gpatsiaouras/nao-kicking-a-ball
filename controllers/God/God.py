from controller import Supervisor
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import math
import numpy as np
import pickle
import os, sys, random

class God:
    popolation_folder = "../../data/popolation/"
    individual_prename = "ind_"
    individual_file_format = ".motion"
    max_generation = 100 # TODO: not used yet

    n_popolation = 100
    n_to_save = 10

    # motion file info/settings
    column_to_evolve = [6,7,8,9,10,11]

    def __init__(self):
        print("God: I am the Alpha and the Omega")
        self.popolate()
        self.scores = [0] * self.n_popolation
        self.current_individual = 0
        self.n_generation = 0

    # are you passing to the next generation? then you should reinitialize some variables...
    def next_gen(self, selected):
        self.n_generation += 1
        self.current_individual = 1

        scores = self.scores # backup
        self.scores = [0] * self.n_popolation # reset to zero the score array

        # Now I put the score back to the selected ones, so they will not be retested!
        for sel in selected:
            self.scores[sel] = scores[sel]
        print ("The Generation #"+str(self.n_generation)+" is ready!")

    # select the top individuals
    def selection(self):
        print("The top one is the #"+str(self.scores.index(max(self.scores)))+" scored: " + str( max(self.scores) ) )
        # the top n_to_save, return they indeces
        return np.argpartition(self.scores, -self.n_to_save)[-self.n_to_save:]

    # kill the individual not selected, and repopolate
    def repopolate(self, selected):
        print("God: Have a lot of children and grandchildren.")
        for i in range(self.n_popolation):
            # I just skipt the number of the ones selected, so I don't have to rename or think to much about it
            if i in selected:
                continue

            # TODO CREATE THE NEW INDIVIDUALS FROM THE SELECTED ONES, HERE! : S
            self.create_individual(self.create_individual_filepath(random.choice(selected)), i)

        self.next_gen(selected)


    # First initialization of the popolation
    def popolate(self):
        print("God: Let the waters bring forth abundantly the moving creature that hath life")
        adamo_filepath = "../../motions/Shoot.motion"
        for i in range(self.n_popolation):
            self.create_individual(adamo_filepath, i)


    def create_individual(self, start_filename, i):
        adamo = self.open_motion_file(start_filename)

        probability_to_change = 0.1
        max_percentuage_to_change = 0.10 # 0.05 = 5%
        min_change = 0.01

        for i_row in range(1, len(adamo)):
            for col in self.column_to_evolve:
                if random.uniform(0,1) < probability_to_change:
                    # ok let's change it then...
                    change = max_percentuage_to_change * float(adamo[i_row][col])
                    if change < min_change:
                        change = min_change

                    # maybe it is negative..
                    if random.uniform(0, 1) > 0.5:
                        change = -change

                    # return in str, because it is much easier to save if is in string!
                    adamo[i_row][col] =  str(float(adamo[i_row][col]) + change)

        self.close_motion_file(adamo, self.popolation_folder+self.individual_prename+str(i)+self.individual_file_format)

    def delete_not_selected(self, selected):
        print("God: The end is now upon you.")

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
        self.scores[self.current_individual-1] = score
        
    def wake_up(self):
        # print("God: Sacrifice your first born in my name!")
        self.current_individual += 1

        # I don't want to retest the ones already tested:
        while self.current_individual <= self.n_popolation and self.scores[self.current_individual-1] != 0:
            self.current_individual += 1

        # Have we finished to evaluate the current popolation?
        if self.current_individual > self.n_popolation:
            print("God: Noah! Prepare the ark!")

            # now I should select and repopulate
            selected = self.selection()
            self.delete_not_selected(selected)
            self.repopolate(selected)

    # get the path to the next motion file to use
    def next_motion_filename(self):
        return self.create_individual_filepath(self.current_individual-1)
    def create_individual_filepath(self, i):
        return self.popolation_folder + self.individual_prename + str(i) + self.individual_file_format

    # it open a motion file, and format in an array rows x cols, remember that the first row is the title
    def open_motion_file(self, filepath):
        # first line is the columns' titles...
        with open(filepath) as f:
            content = f.readlines()

        # return np.array([x.strip().split(",") for x in content])
        # return [ float(x) for x in [x.strip().split(",") for x in content]]
        return [x.strip().split(",") for x in content]

    # content = [x.strip() for x in content]
    # _content = []
    # for c in content:
    #     _content.append(c.split(","))

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
                print("Failed to load the last god : S")
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
    # def saveGod(self):
    #     np.save(self.godFile, self.god)
    # def loadGod(self):
    #     self.godFile = np.load(self.godFile)

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
