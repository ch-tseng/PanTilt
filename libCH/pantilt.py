# How  to use:
#  1. Import the library
#	from libCH.pantilt import PanTilt
#
#  2. Create PanTilt object, first parameter is the pin number of PAN servo, the second is Tilt pin number
#	motorPT = PanTilt(12,11)
#
#  3. Start to send PWM to PAN/Tilt servos(you will hear that the motor is turning) 
#	motorPT.start()
#
#  4. Move right (or left) for X length ( 1 is about 18 degrees, if you want to turn right for 90 degrees,
#     the value for the parameter is 90, if you want to turn left for 90 degrees, the value is -90)
#	motorPT.movePAN(X)
#
#  5. Turn up or down for Y length ( 1 is about 18 degrees, if you want to move up for 10 degrees,
#     the value for the parameter is 0.56, if you want to turn down for 90 degrees, the value is -90)
#		motorPT.moveTILT(Y)
#
#  6. Turn right(or left) to xDegree position( xDegree's range is 2.5~12.5, corresponding to 0~180 degree,
#     so, if the value for the parameter is 7.5, the servoo will turn to center position(90 degree))
#               motorPT.movePAN(xDegree)
#
#  7. Turn up(or down) to the yDegree position( yDegree's range is 2.5~12.5, corresponding to 0~180 degree,
#      so, if the value for the parameter is 7.5, the servoo will turn to center position(90 degree))
#		motorPT.moveTILT(xDegree)
#
#  8. Stop sending PWM to Pan/Tilt (The servos of PAN/TILT will be stopped)
#		motorPT.stop()
#



import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

minPANangle = 2.5
minTILTangle = 2.5
maxPANangle = 12.5
maxTILTangle = 12.5

class PanTilt:

    def __init__(self, pinPAN, pinTILT, initAnglePAN=7.50, initAngleTilt=7.50):
        self.pinPAN = pinPAN
        self.pinTILT = pinTILT
        self.initAnglePAN = initAnglePAN
        self.nowAnglePAN = initAnglePAN
        self.lastAnglePAN = 0
        self.initAngleTILT = initAngleTilt
        self.nowAngleTILT = initAngleTilt
        self.lastAngleTILT = 0
        self.inaction = 0

        GPIO.setup(pinPAN, GPIO.OUT)
        self.pwmPAN = GPIO.PWM(pinPAN, 50)
        #self.pwmPAN.start(initAnglePAN)

        GPIO.setup(pinTILT, GPIO.OUT)
        self.pwmTILT = GPIO.PWM(pinTILT, 50)
        #self.pwmTILT.start(initAngleTilt)

    def movePAN(self, angle=0.5):
        self.nowAnglePAN = self.nowAnglePAN + angle
        if self.nowAnglePAN>maxPANangle: self.nowAnglePAN=maxPANangle
        if self.nowAnglePAN<minPANangle: self.nowAnglePAN=minPANangle

        if self.nowAnglePAN!=self.lastAnglePAN:
            #print("move PAN from " + str(self.lastAnglePAN) + " to " +str(self.nowAnglePAN))
            self.pwmPAN.ChangeDutyCycle(self.nowAnglePAN)  # turn towards 90 degree
            #self.pwmPAN.start(self.nowAnglePAN)  # turn towards 90 degree
            self.lastAnglePAN = self.nowAnglePAN

            #self.pwmPAN.stop(self.nowAnglePAN)

    def moveTILT(self, angle=0.5):
        self.nowAngleTILT = self.nowAngleTILT + angle
        if self.nowAngleTILT>maxTILTangle: self.nowAngleTILT = maxTILTangle
        if self.nowAngleTILT<minTILTangle: self.nowAngleTILT = minTILTangle

        if self.nowAngleTILT!=self.lastAngleTILT:
            #print("move TILT from " + str(self.lastAngleTILT) + " to " +str(self.nowAngleTILT))
            self.pwmTILT.ChangeDutyCycle(self.nowAngleTILT)  # turn towards 90 degree
            #self.pwmTILT.start(self.nowAngleTILT)
            self.lastAngleTILT = self.nowAngleTILT

            #self.pwmTILT.stop(self.nowAngleTILT)

    def movePANto(self, angle=7.5):
        if angle<minPANangle: angle=minPANangle
        if angle>maxPANangle: angle=maxPANangle
        self.nowAnglePAN = angle

        if angle!=self.lastAnglePAN:
            #print("move PAN from " + str(self.lastAnglePAN) + " to " +str(angle))
            self.pwmPAN.ChangeDutyCycle(angle)  # turn towards 90 degree
            #self.pwmPAN.start(angle)
            self.lastAnglePAN = angle

            #self.pwmPAN.stop(angle)

    def moveTILTto(self, angle=7.5):
        if angle<minTILTangle: angle=minTILTangle
        if angle>maxTILTangle: angle=maxTILTangle
        self.nowAngleTILT = angle

        if angle!=self.lastAngleTILT:
            #print("move TILT from " + str(self.lastAngleTILT) + " to " +str(angle))
            self.pwmTILT.ChangeDutyCycle(angle)  # turn towards 90 degree
            #self.pwmTILT.start(angle)
            self.lastAngleTILT = angle

            #self.pwmTILT.stop(angle)

    def stop(self):
        self.inaction = 0
        self.stopTILT()
        self.stopPAN()
        #GPIO.cleanup()

    def start(self):
        self.inaction = 1
        GPIO.setmode(GPIO.BOARD)
        self.startPAN()
        self.startTILT()

    def stopPAN(self):
       self.pwmPAN.stop()

    def stopTILT(self):
       self.pwmTILT.stop()

    def startPAN(self):
        GPIO.setup(self.pinPAN, GPIO.OUT)
        self.pwmPAN = GPIO.PWM(self.pinPAN, 50)
        self.pwmPAN.start(self.nowAnglePAN)

    def startTILT(self):
        GPIO.setup(self.pinTILT, GPIO.OUT)
        self.pwmTILT = GPIO.PWM(self.pinTILT, 50)
        self.pwmTILT.start(self.nowAngleTILT)

