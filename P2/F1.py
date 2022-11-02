import time

import numpy as np
import cv2 as cv

from GUI import GUI
from HAL import HAL


class pid_controller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.error = 0
        self.previous_error = 0
        self.integral = 0
        self.derivative = 0

    def update(self, error):
        self.error = error
        #if the error changes sign, reset the integral
        #if self.error * self.previous_error < 0:
        #    self.integral = 0
        
        self.integral = self.integral + self.error
        self.derivative = self.error - self.previous_error
        self.previous_error = self.error

        return self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative


def detect_line(frame,height):
    """
    Detects the line in the frame and returns the x coordinate of the center of the line
    """
    #instead of masking, we can just crop the image
    masked=frame[height:height+1,0:frame.shape[1]]
    # convert to hsv
    masked= cv.cvtColor(~masked, cv.COLOR_BGR2HSV)

    # detect red color in negated hsv image
    masked = cv.inRange(masked, (90 - 10, 70, 50), (90 + 10, 255, 255))
    # detect center of the white pixels in masked image
    M = cv.moments(masked)
    # calculate x,y coordinate of center
    cX = int(M["m10"] / M["m00"])
    #cY = int(M["m01"] / M["m00"])

    # draw the center of the shape on the image
    cv.circle(frame, (cX, height), 3, (0, 0, 0), -1)
    #draw a line at height
    cv.line(frame,(0,height),(frame.shape[1],height),(0,0,0),1)

    return cX

def calculate_errors(frame):
    """
    Calculates the errors for the PID controller
    """
    height1=int(frame.shape[0]/1.7)
    goal_turn=0.54*frame.shape[1]

    height2=int(frame.shape[0]/1.8)
    
    height3=int(frame.shape[0]/1.9)
    goal_speed=0.51*frame.shape[1]
    # detect line
    try:
        x1 = detect_line(frame,height1) #lower line
        x2 = detect_line(frame,height2) #middle line
        x3 = detect_line(frame,height3) #upper line
    except:
        print("error: detect_line")
        return 0,True
    # draw vertical line at goal
    cv.line(frame,(int(goal_turn),height1),(int(goal_turn),frame.shape[0]),(0,0,0),1)
    #cv.line(frame,(int(goal_speed),0),(int(goal_speed),height2),(0,0,0),1)
    # calculate error, distance between center of line and goal
    turn_error = (goal_turn - x1) / (frame.shape[1]/2)
    #speed_error= abs(goal_speed-x2)/frame.shape[1]
    #detect if x1 x2 and x3 are alineated
    is_curve=abs((x1-x2)-(x2-x3))+abs(turn_error*3)
    return turn_error, is_curve

v_robot=0
w_robot=0
pid_turn = pid_controller(1.5,0,7)
#pid_turn = pid_controller(1.5,0,0)

"""
MAX_STRAIGHT_SPEED=7 #if you want speed
MAX_CURVE_SPEED=4 #if you want speed

MAX_STRAIGHT_SPEED=6 #if you want reliability
MAX_CURVE_SPEED=3 #if you want reliability
12,8,3
"""
MAX_STRAIGHT_SPEED=11
FAST_CURVE_SPEED=8
CURVE_SPEED=3


HAL.setV(0)
HAL.setW(0)
time.sleep(3)

# simulator loop
while True:    
    # get frame from simulator
    frame=HAL.getImage()
    # calculate errors for PID controller
    turn_error, is_curve=calculate_errors(frame)
    w_robot=pid_turn.update(turn_error)

    #instead of using the speed error, we can use a pid controller for the speed
    v_robot=min(MAX_STRAIGHT_SPEED,max(MAX_STRAIGHT_SPEED-pid_speed.update(speed_error),CURVE_SPEED))    
    # send commands to simulator
    GUI.showImage(frame)
    HAL.setV(v_robot)
    HAL.setW(w_robot)
    #this loop should execute as fast as possible, avoid prints, delays and optimize the code

"""
#opencv tests
#measure the time it takes to process one frame
frame = cv.imread(cv.samples.findFile("Screenshot-5.png"))
time_init=time.time()
turn_error, speed_error, is_curve=calculate_errors(frame)
time_end=time.time()
print("time:",time_end-time_init)
cv.imshow("frame", frame)
print("turn_error:",turn_error)
print("speed_error:",speed_error)
#wait for key press
cv.waitKey(0)
cv.destroyAllWindows()
"""