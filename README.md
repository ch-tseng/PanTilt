# PanTilt
# How  to use PanTilt library:
#
#  1. Import the library
#       from libCH.pantilt import PanTilt
#
#  2. Create PanTilt object, first parameter is the pin number of PAN servo, the second is Tilt pin number
#       motorPT = PanTilt(12,11)
#
#  3. Start to send PWM to PAN/Tilt servos(you will hear that the motor is turning)
#       motorPT.start()
#
#  4. Move right (or left) for X length ( 1 is about 18 degrees, if you want to turn right for 90 degrees,
#     the value for the parameter is 90, if you want to turn left for 90 degrees, the value is -90)
#       motorPT.movePAN(X)
#
#  5. Turn up or down for Y length ( 1 is about 18 degrees, if you want to move up for 10 degrees,
#     the value for the parameter is 0.56, if you want to turn down for 90 degrees, the value is -90)
#               motorPT.moveTILT(Y)
#
#  6. Turn right(or left) to xDegree position( xDegree's range is 2.5~12.5, corresponding to 0~180 degree,
#     so, if the value for the parameter is 7.5, the servoo will turn to center position(90 degree))
#               motorPT.movePAN(xDegree)
#
#  7. Turn up(or down) to the yDegree position( yDegree's range is 2.5~12.5, corresponding to 0~180 degree,
#      so, if the value for the parameter is 7.5, the servoo will turn to center position(90 degree))
#               motorPT.moveTILT(xDegree)
#
#  8. Stop sending PWM to Pan/Tilt (The servos of PAN/TILT will be stopped)
#               motorPT.stop()
#

