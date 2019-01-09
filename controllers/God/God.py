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


class Driver (Supervisor):
    timeStep = 128

    def initialization(self):
        self.nao = self.getFromDef('NAO')
        self.t_nao = self.nao.getField('translation')
        self.ball = self.getFromDef('Ball')
        self.t_ball = self.ball.getField('translation')
        self.keyboard.enable(self.timeStep)
        self.keyboard = self.getKeyboard()

    def run(self):
        self.displayHelp()

        # Main loop.
        while True:
            # Deal with the pressed keyboard key.
            k = self.keyboard.getKey()
            if k == ord('N'):
                translationValues = self.t_nao.getSFVec3f()
                print(translationValues)
                translationValues = self.t_ball.getSFVec3f()
                print(translationValues)

            # Perform a simulation step, quit the loop when
            # Webots is about to quit.
            if self.step(self.timeStep) == -1:
                break

    def displayHelp(self):
        print(
            'Commands:\n'
            ' N for position informations\n'
        )


controller = Driver()
controller.initialization()
controller.run()
