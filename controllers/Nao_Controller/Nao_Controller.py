from controller import Robot, Accelerometer, Camera, DistanceSensor, \
                       GPS, Gyro, InertialUnit, Keyboard, LED, Motion, \
                       Motor, TouchSensor
import sys

# this is the main class
class Nao (Robot):
    PHALANX_MAX = 8


    def receive_motion_file(self):
        if self.receiver.getQueueLength() > 0:
            message = self.receiver.getData().decode('utf-8')
            if message == "Sacrifice yourself in My Name!":
                print("This wasn't real anyway... ( Nao Controller exited )")
                sys.exit(0)
            return message
        else:
            print("No signs from God... does she exist?")
            return False



    def startMotion(self, motion):
        # interrupt current motion
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()

        # start new motion
        motion.play()
        self.currentlyPlaying = motion

    # # load motion files
    # def loadMotionFiles(self):
    #     self.handWave = Motion('../../motions/HandWave.motion')
    #     self.forwards = Motion('../../motions/Forwards50.motion')
    #     self.backwards = Motion('../../motions/Backwards.motion')
    #     self.sideStepLeft = Motion('../../motions/SideStepLeft.motion')
    #     self.sideStepRight = Motion('../../motions/SideStepRight.motion')
    #     self.turnLeft60 = Motion('../../motions/TurnLeft60.motion')
    #     self.turnRight60 = Motion('../../motions/TurnRight60.motion')
    #     self.shoot2x = Motion('../../motions/Shoot2x.motion')
    # # the accelerometer axes are oriented as on the real robot
    # # however the sign of the returned values may be opposite
    # def printAcceleration(self):
    #     acc = self.accelerometer.getValues()
    #     print('----------accelerometer----------')
    #     print('acceleration: [ x y z ] = [%f %f %f]' % (acc[0], acc[1], acc[2]))
    #
    # # the gyro axes are oriented as on the real robot
    # # however the sign of the returned values may be opposite
    # def printGyro(self):
    #     vel = self.gyro.getValues()
    #     print('----------gyro----------')
    #     # z value is meaningless due to the orientation of the Gyro
    #     print('angular velocity: [ x y ] = [%f %f]' % (vel[0], vel[1]))
    #
    # def printGps(self):
    #     p = self.gps.getValues()
    #     print('----------gps----------')
    #     print('position: [ x y z ] = [%f %f %f]' % (p[0], p[1], p[2]))
    #
    # # the InertialUnit roll/pitch angles are equal to naoqi's AngleX/AngleY
    # def printInertialUnit(self):
    #     rpy = self.inertialUnit.getRollPitchYaw()
    #     print('----------inertial unit----------')
    #     print('roll/pitch/yaw: [%f %f %f]' % (rpy[0], rpy[1], rpy[2]))
    #
    # def printFootSensors(self):
    #     newtons = 0.0
    #     fsv = [] # force sensor values
    #
    #     fsv.append(self.fsr[0].getValues())
    #     fsv.append(self.fsr[1].getValues())
    #
    #     l = []
    #     r = []
    #
    #     newtonsLeft = 0
    #     newtonsRight = 0
    #
    #     # The coefficients were calibrated against the real
    #     # robot so as to obtain realistic sensor values.
    #     l.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] + 1.15 * fsv[0][1]) # Left Foot Front Left
    #     l.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] - 1.15 * fsv[0][1]) # Left Foot Front Right
    #     l.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] - 1.15 * fsv[0][1]) # Left Foot Rear Right
    #     l.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] + 1.15 * fsv[0][1]) # Left Foot Rear Left
    #
    #     r.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] + 1.15 * fsv[1][1]) # Right Foot Front Left
    #     r.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] - 1.15 * fsv[1][1]) # Right Foot Front Right
    #     r.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] - 1.15 * fsv[1][1]) # Right Foot Rear Right
    #     r.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] + 1.15 * fsv[1][1]) # Right Foot Rear Left
    #
    #     for i in range(0, len(l)):
    #         l[i] = max(min(l[i], 25), 0)
    #         r[i] = max(min(r[i], 25), 0)
    #         newtonsLeft += l[i]
    #         newtonsRight += r[i]
    #
    #     print('----------foot sensors----------')
    #     print('+ left ---- right +')
    #     print('+-------+ +-------+')
    #     print('|'  + str(round(l[0],1)) + \
    #           '  ' + str(round(l[1],1)) + \
    #           '| |'+ str(round(r[0],1)) + \
    #           '  ' + str(round(r[1],1)) + \
    #           '|  front')
    #     print('| ----- | | ----- |')
    #     print('|'  + str(round(l[3],1)) + \
    #           '  ' + str(round(l[2],1)) + \
    #           '| |'+ str(round(r[3],1)) + \
    #           '  ' + str(round(r[2],1)) + \
    #           '|  back')
    #     print('+-------+ +-------+')
    #     print('total: %f Newtons, %f kilograms' \
    #           % ((newtonsLeft + newtonsRight), ((newtonsLeft + newtonsRight)/9.81)))
    #
    # def printFootBumpers(self):
    #     ll = self.lfootlbumper.getValue()
    #     lr = self.lfootrbumper.getValue()
    #     rl = self.rfootlbumper.getValue()
    #     rr = self.rfootrbumper.getValue()
    #     print('----------foot bumpers----------')
    #     print('+ left ------ right +')
    #     print('+--------+ +--------+')
    #     print('|'  + str(ll) + '  ' + str(lr) + '| |'+ str(rl) + '  ' + str(rr) + '|')
    #     print('|        | |        |')
    #     print('|        | |        |')
    #     print('+--------+ +--------+')
    #
    # def printUltrasoundSensors(self):
    #     dist = []
    #     for i in range(0, len(self.us)):
    #         dist.append(self.us[i].getValue())
    #
    #     print('-----ultrasound sensors-----')
    #     print('left: %f m, right %f m' % (dist[0], dist[1]))
    #
    # def printCameraImage(self, camera):
    #     scaled = 2 # defines by which factor the image is subsampled
    #     width = camera.getWidth()
    #     height = camera.getHeight()
    #
    #     # read rgb pixel values from the camera
    #     image = camera.getImage()
    #
    #     print('----------camera image (gray levels)---------')
    #     print('original resolution: %d x %d, scaled to %d x %f' \
    #           % (width, height, width/scaled, height/scaled))
    #
    #     for y in range(0, height/scaled):
    #         line = ''
    #         for x in range(0, width/scaled):
    #             gray = camera.imageGetGray(image, width, x * scaled, y * scaled) * 9 / 255 # between 0 and  instead of 0 and 255
    #             line = line + str(int(gray))
    #         print(line)
    #
    # def setAllLedsColor(self, rgb):
    #     # these leds take RGB values
    #     for i in range(0, len(self.leds)):
    #         self.leds[i].set(rgb)
    #
    #     # ear leds are single color (blue)
    #     # and take values between 0 - 255
    #     self.leds[5].set(rgb & 0xFF)
    #     self.leds[6].set(rgb & 0xFF)
    #
    # def setHandsAngle(self, angle):
    #     for i in range(0, self.PHALANX_MAX):
    #         clampedAngle = angle
    #         if clampedAngle > self.maxPhalanxMotorPosition[i]:
    #             clampedAngle = self.maxPhalanxMotorPosition[i]
    #         elif clampedAngle < self.minPhalanxMotorPosition[i]:
    #             clampedAngle = self.minPhalanxMotorPosition[i]
    #
    #         if len(self.rphalanx) > i and self.rphalanx[i] is not None:
    #             self.rphalanx[i].setPosition(clampedAngle)
    #         if len(self.lphalanx) > i and self.lphalanx[i] is not None:
    #             self.lphalanx[i].setPosition(clampedAngle)
    #
    #
    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.timeStep = int(self.getBasicTimeStep())

        # # gps
        # self.gps = self.getGPS('gps')
        # self.gps.enable(4 * self.timeStep)
        #
        # # camera
        # self.cameraTop = self.getCamera("CameraTop")
        # self.cameraBottom = self.getCamera("CameraBottom")
        # self.cameraTop.enable(4 * self.timeStep)
        # self.cameraBottom.enable(4 * self.timeStep)
        #
        # # accelerometer
        # self.accelerometer = self.getAccelerometer('accelerometer')
        # self.accelerometer.enable(4 * self.timeStep)
        #
        # # gyro
        # self.gyro = self.getGyro('gyro')
        # self.gyro.enable(4 * self.timeStep)
        #
        # # inertial unit
        # self.inertialUnit = self.getInertialUnit('inertial unit')
        # self.inertialUnit.enable(self.timeStep)
        #
        # # ultrasound sensors
        # self.us = []
        # usNames = ['Sonar/Left','Sonar/Right']
        # for i in range(0, len(usNames)):
        #     self.us.append(self.getDistanceSensor(usNames[i]))
        #     self.us[i].enable(self.timeStep)
        #
        # # foot sensors
        # self.fsr = []
        # fsrNames = ['LFsr', 'RFsr']
        # for i in range(0, len(fsrNames)):
        #     self.fsr.append(self.getTouchSensor(fsrNames[i]))
        #     self.fsr[i].enable(self.timeStep)
        #
        # # foot bumpers
        # self.lfootlbumper = self.getTouchSensor('LFoot/Bumper/Left')
        # self.lfootrbumper = self.getTouchSensor('LFoot/Bumper/Right')
        # self.rfootlbumper = self.getTouchSensor('RFoot/Bumper/Left')
        # self.rfootrbumper = self.getTouchSensor('RFoot/Bumper/Right')
        # self.lfootlbumper.enable(self.timeStep)
        # self.lfootrbumper.enable(self.timeStep)
        # self.rfootlbumper.enable(self.timeStep)
        # self.rfootrbumper.enable(self.timeStep)
        #
        # # there are 7 controlable LED groups in Webots
        # self.leds = []
        # self.leds.append(self.getLED('ChestBoard/Led'))
        # self.leds.append(self.getLED('RFoot/Led'))
        # self.leds.append(self.getLED('LFoot/Led'))
        # self.leds.append(self.getLED('Face/Led/Right'))
        # self.leds.append(self.getLED('Face/Led/Left'))
        # self.leds.append(self.getLED('Ears/Led/Right'))
        # self.leds.append(self.getLED('Ears/Led/Left'))
        #
        # # get phalanx motor tags
        # # the real Nao has only 2 motors for RHand/LHand
        # # but in Webots we must implement RHand/LHand with 2x8 motors
        # self.lphalanx = []
        # self.rphalanx = []
        # self.maxPhalanxMotorPosition = []
        # self.minPhalanxMotorPosition = []
        # for i in range(0, self.PHALANX_MAX):
        #     self.lphalanx.append(self.getMotor("LPhalanx%d" % (i + 1)))
        #     self.rphalanx.append(self.getMotor("RPhalanx%d" % (i + 1)))
        #
        #     # assume right and left hands have the same motor position bounds
        #     self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
        #     self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())
        #
        # # shoulder pitch motors
        # self.RShoulderPitch = self.getMotor("RShoulderPitch");
        # self.LShoulderPitch = self.getMotor("LShoulderPitch");
        #
        # # keyboard
        # self.keyboard = self.getKeyboard()
        # self.keyboard.enable(10 * self.timeStep)

    def printHelp(self):
        print('----------Nao_Controller Loaded ----------')
    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = False


        # initialize stuff
        self.findAndEnableDevices()
        # self.receive_motion_file()

        # to receive messages:
        self.receiver = self.getReceiver('receiver')
        self.receiver.enable(self.timeStep)

        self.printHelp()

    def run(self):
        # self.handWave.setLoop(True)
        # self.handWave.play()

        # until a key is pressed
        # key = -1
        # while robot.step(self.timeStep) != -1:
            # key = self.keyboard.getKey()
            # if key > 0:
                # break

        # self.handWave.setLoop(False)


        motion_file = False
        played = False
        while True:
            if not motion_file:
                motion_file = self.receive_motion_file()
            if motion_file and not played:
                self.startMotion(Motion(motion_file))
                played = True

            if robot.step(self.timeStep) == -1:
                break


# create the Robot instance and run main loop
robot = Nao()
robot.run()
