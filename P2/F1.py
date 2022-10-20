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


def detect_line(frame):
    mask = np.zeros_like(frame)# Create mask
    # draw a line horizontal on the mask
    height=int(frame.shape[0]/1.7)
    cv.line(mask, (0, height), (frame.shape[1], height), (255, 255, 255), 4)
    # apply the mask
    masked = cv.bitwise_and(frame, mask)
    # convert to hsv
    masked = cv.cvtColor(masked, cv.COLOR_BGR2HSV)
    # detect red color in hsv image
    lower_red = np.array([-10, 120, 70])
    upper_red = np.array([10, 255, 255])
    masked = cv.inRange(masked, lower_red, upper_red)
    # detect center of the white pixels in masked image
    M = cv.moments(masked)
    # calculate x,y coordinate of center
    try:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    except:
        cX = 0
        cY = 0
    # draw the center of the shape on the image
    cv.circle(frame, (cX, cY), 7, (255, 0, 255), -1)

    #cv.imshow("frame", frame)
    #cv.imshow("mask", mask)
    #cv.imshow("masked", masked)

    #error is the distance from the center of the image to the center of the line
    #error is a percentage of the image width
    error = (frame.shape[1]/2 - cX) / (frame.shape[1]/2)
    return error, frame

v_robot=0
w_robot=0
pid_turn = pid_controller(0.5, 0.5, 0)

while True:
    time.sleep(0.1)
    HAL.setV(v_robot)

    #w_robot should be limited to a maximum value
    HAL.setW(min(0.1, max(-0.1, w_robot)))


    HAL.setW(w_robot)

    #frame = cv.imread(cv.samples.findFile("Screenshot-5.png"))
    
    frame=HAL.getImage()
    error, frame = detect_line(frame)
    w_robot=pid_turn.update(error)
    print(w_robot, error)
    GUI.showImage(frame)

