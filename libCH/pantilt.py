import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

minPANangle = 2.5
minTILTangle = 2.5
maxPANangle = 12.5
maxTILTangle = 12.5

class PanTilt:

    def __init__(self, pinPAN, pinTILT, initAnglePAN=7.50, initAngleTilt=7.50):
        self.pinPan = pinPAN
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
        self.pwmPAN.start(initAnglePAN)

        GPIO.setup(pinTILT, GPIO.OUT)
        self.pwmTILT = GPIO.PWM(pinTILT, 50)
        self.pwmTILT.start(initAngleTilt)

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
        self.stopTILT()
        self.stopPAN()

    def start(self):
        self.startPAN()
        self.startTILT()

    def stopPAN(self):
            self.pwmPAN.stop()

    def stopTILT(self):
            self.pwmTILT.stop()

    def startPAN(self):
        self.pwmPAN.start(self.nowAnglePAN)


    def startTILT(self):
        self.pwmTILT.start(self.nowAngleTILT)

