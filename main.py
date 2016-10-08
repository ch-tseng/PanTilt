# import the necessary packages
from imutils.video import VideoStream
import datetime
import argparse
import imutils
import time
import cv2
import numpy
import RPi.GPIO as GPIO
from libCH.pantilt import PanTilt

GPIO.setmode(GPIO.BOARD)

imgsize_w = int(640*0.5)
imgsize_h = int(480*0.5)
moveDegree = 0.25


# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--picamera", type=int, default=-1,
	help="whether or not the Raspberry Pi camera should be used")
args = vars(ap.parse_args())

#_[LED_CONFIGURATION]__________________________________________
pinLED = 36
GPIO.setup(pinLED ,GPIO.OUT)

#_[PIR_CONFIGURATION]__________________________________________
pinPIR = 32
GPIO.setup(pinPIR ,GPIO.IN)

#_[PAN/TILT_SERVO_CONFIGURATION]_______________________________
motorPT = PanTilt(12,11)
# move to center (90 degrees)
motorPT.movePANto(7.5)
motorPT.moveTILTto(7.5)


# initialize the video stream and allow the cammera sensor to warmup
vs = VideoStream(usePiCamera=1).start()
time.sleep(2.0)

#_INTERRUPTS_FOR_PIR___________________________________________
def MOTION(pinPIR):
    global statusPIR
    print("PIR detected!")
    #searchAround()
    #motorPT.startPAN()
    #motorPT.startTILT()
    statusPIR = 1

def movePANTILT(diffX, diffY):
    global moveDegree, imgsize_w, imgsize_h
    adjustDegreeX = moveDegree
    adjustDegreeY = moveDegree
    lengthX1 = (imgsize_w/2) * 0.2
    lengthY1 = (imgsize_h/2) * 0.2
    lengthX2 = (imgsize_w/2) * 0.6
    lengthY2 = (imgsize_h/2) * 0.6
    lengthX3 = (imgsize_w/2) * 0.8
    lengthY3 = (imgsize_h/2) * 0.8
    lengthX4 = (imgsize_w/2) * 0.9
    lengthY4 = (imgsize_h/2) * 0.9


    if abs(diffX)<lengthX1: adjustDegreeX = moveDegree/2
    if abs(diffX)<lengthX2 and abs(diffX)>=lengthX1: adjustDegreeX = moveDegree/1.5
    if abs(diffX)<lengthX3 and abs(diffX)>=lengthX2: adjustDegreeX = moveDegree
    if abs(diffX)<lengthX4 and abs(diffX)>=lengthX3: adjustDegreeX = moveDegree*2.0
    if abs(diffX)>=lengthX4: adjustDegreeX = moveDegree*2.5

    if abs(diffY)<lengthY1: adjustDegreeY = moveDegree/2
    if abs(diffY)<lengthY2 and abs(diffY)>=lengthY1: adjustDegreeY = moveDegree/1.5
    if abs(diffY)<lengthY3 and abs(diffY)>=lengthY2: adjustDegreeY = moveDegree
    if abs(diffY)<lengthY4 and abs(diffY)>=lengthY3: adjustDegreeY = moveDegree*2.0
    if abs(diffY)>=lengthY4: adjustDegreeY = moveDegree*2.5

    if diffX<0:
        motorPT.movePAN(-adjustDegreeX)
        print("Move X --> " + str(-adjustDegreeX))
    else:
        motorPT.movePAN(adjustDegreeX)
        print("Move X --> " + str(-adjustDegreeX))

    if diffY<0:
        motorPT.moveTILT(adjustDegreeY)
        print("Move Y --> " + str(adjustDegreeY))
    else:
        motorPT.moveTILT(-adjustDegreeY)
        print("Move Y --> " + str(-adjustDegreeY))



GPIO.add_event_detect(pinPIR, GPIO.RISING, callback=MOTION)

i=0
statusPIR = 0


# loop over the frames from the video stream
while True:
        i+=1
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 400 pixels
	frame = vs.read()
	frame = imutils.resize(frame, width=imgsize_w)
        frame = imutils.rotate(frame, 180)
	# draw the timestamp on the frame
	#timestamp = datetime.datetime.now()
	#ts = timestamp.strftime("%A %d %B %Y %I:%M:%S%p")
	#cv2.putText(frame, ts, (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
	#	0.35, (0, 0, 255), 1)
        face_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml')
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.1, 5)

        iface = 0
        for (x,y,w,h) in faces:
            if iface == 0:
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
            else:
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),1)
            iface += 1

        print str(i) + ") Found "+str(len(faces))+" face(s)"


	# show the frame
	#cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

        if(len(faces)>0):
            GPIO.output(pinLED, GPIO.HIGH)
            xFace = faces[0][0]
            yFace = faces[0][1]

            #print "x,y = " + str(xFace) + "," + str(yFace)
            diffX = (imgsize_w/2) - x
            diffY = (imgsize_h/2) - y
            print "diffX, diffY = " + str(diffX) + "," + str(diffY)
            movePANTILT(diffX, diffY)

        else:
            GPIO.output(pinLED, GPIO.LOW)

        statusPIR = GPIO.input(pinPIR)
        print("PIR="+str(statusPIR))


	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
