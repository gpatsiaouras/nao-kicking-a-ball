# Copyright 1996-2018 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This controller gives to its node the following behavior:
Listen the keyboard. According to the pressed key, send a
message through an emitter or handle the position of Robot1.
"""

from controller import Supervisor
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import math


class Driver (Supervisor):
    timeStep = 128
    football_goal = Polygon([(4.57151, 0.8), (4.57151, -0.8), (5.04, -0.8), (5.04, 0.8)])
    goal = False

    def initialization(self):
        self.nao = self.getFromDef('NAO')
        self.t_nao = self.nao.getField('translation')
        self.ball = self.getFromDef('Ball')
        self.t_ball = self.ball.getField('translation')
        self.keyboard.enable(self.timeStep)
        self.keyboard = self.getKeyboard()
        
        # Store initial coordinates of ball
        self.ballX = self.ball.getPosition()[0] # 0 is x
        self.ballZ = self.ball.getPosition()[2] # 2 is z

    def run(self):
        self.displayHelp()

        # Main loop.
        while True:
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
            
            self.calculate_velocity()

            if not self.goal:
                self.is_goal()
            
            if self.step(self.timeStep) == -1:
                break

    def displayHelp(self):
        print(
            'Commands:\n'
            ' N for position informations\n'
        )
    
    def calculate_velocity(self):
        diffX = math.fabs(self.ball.getPosition()[0] - self.ballX)
        diffZ = math.fabs(self.ball.getPosition()[2] - self.ballZ)

        difference = math.sqrt(math.pow(diffX, 2) - math.pow(diffZ, 2))
        print("Ball moved: " + str(difference))

        # Assign new position
        self.ballX = self.ball.getPosition()[0]
        self.ballZ = self.ball.getPosition()[2]
        
        
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
        

controller = Driver()
controller.initialization()
controller.run()
