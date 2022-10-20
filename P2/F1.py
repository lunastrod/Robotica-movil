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
        self.integral = self.integral + self.error
        self.derivative = self.error - self.previous_error
        self.previous_error = self.error
        return self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative


def detect_line(frame,height):
    """
    Detects the line in the frame and returns the x coordinate of the center of the line
    """
    #instead of masking, we can just crop the image
    masked=frame[height:height+5,0:frame.shape[1]]
    # convert to hsv
    masked = cv.cvtColor(masked, cv.COLOR_BGR2HSV)
    # detect red color in hsv image
    lower_red = np.array([0, 120, 180])
    upper_red = np.array([10, 255, 255])
    masked = cv.inRange(masked, lower_red, upper_red)
    # detect center of the white pixels in masked image
    M = cv.moments(masked)
    # calculate x,y coordinate of center
    try:
        cX = int(M["m10"] / M["m00"])
        #cY = int(M["m01"] / M["m00"])
    except:
        return 0
    # draw the center of the shape on the image
    cv.circle(frame, (cX, height), 3, (0, 0, 0), -1)
    #draw a line at height
    cv.line(frame,(0,height),(frame.shape[1],height),(0,0,0),1)

    return cX

def calculate_errors(frame):
    """
    Calculates the errors for the PID controller
    """
    height1=int(frame.shape[0]/1.6)
    goal1=0.54*frame.shape[1]
    height2=int(frame.shape[0]/1.9)
    goal2=0.51*frame.shape[1]
    # detect line
    x1 = detect_line(frame,height1) #lower line
    x2 = detect_line(frame,height2) #upper line
    # draw vertical line at goal
    cv.line(frame,(int(goal1),height1),(int(goal1),frame.shape[0]),(0,0,0),1)
    cv.line(frame,(int(goal2),0),(int(goal2),height2),(0,0,0),1)
    # calculate error, distance between center of line and goal
    turn_error = (goal1 - x1) / (frame.shape[1]/2)
    speed_error= abs(goal2-x2)/frame.shape[1]
    return turn_error, speed_error

v_robot=0
w_robot=0
pid_turn = pid_controller(3,0.01,6)
pid_speed = pid_controller(100,0,100)
line_found=0

"""
MAX_STRAIGHT_SPEED=7 #if you want speed
MAX_CURVE_SPEED=4 #if you want speed

MAX_STRAIGHT_SPEED=6 #if you want reliability
MAX_CURVE_SPEED=3 #if you want reliability
"""
MAX_STRAIGHT_SPEED=7
CURVE_SPEED=4


# simulator loop
while True:    
    # get frame from simulator
    frame=HAL.getImage()
    # calculate errors for PID controller
    turn_error, speed_error=calculate_errors(frame)
    w_robot=pid_turn.update(turn_error)
    #instead of using the speed error, we can use a pid controller for the speed
    if(line_found>10):
        v_robot=min(MAX_STRAIGHT_SPEED,max(MAX_STRAIGHT_SPEED-pid_speed.update(speed_error),CURVE_SPEED))
    else: #slow down if line is not found at the beginning of the run
        if(turn_error<0.1 and turn_error>-0.1):
            line_found+=1
        v_robot=0.1
    
    # send commands to simulator
    GUI.showImage(frame)
    HAL.setV(v_robot)
    HAL.setW(w_robot)
    #this loop should execute as fast as possible, avoid prints, delays and optimize the code

"""
#opencv tests
#measure the time it takes to process one frame
frame = cv.imread(cv.samples.findFile("Centered.png"))
time_init=time.time()
turn_error,speed_error = calculate_errors(frame)
time_end=time.time()
print("time:",time_end-time_init)
cv.imshow("frame", frame)
print("turn_error:",turn_error)
print("speed_error:",speed_error)
#wait for key press
cv.waitKey(0)
cv.destroyAllWindows()
"""