import time

import numpy as np
import cv2 as cv

#from GUI import GUI
#from HAL import HAL

"""

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

pid_turn = pid_controller(0.5, 0.5, 0.5)

v_robot=5
w_robot=0.0

"""

#while True:
    #time.sleep(10)# 10Hz
    #HAL.setV(v_robot) 
    #HAL.setW(w_robot)
    #frame=HAL.getImage()
frame = cv.imread(cv.samples.findFile("Screenshot-5.png"))


hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
cv.imshow("frame", frame)
cv.imshow("hsv", hsv)
print("a")
k = cv.waitKey(0)
print("a")
#GUI.showImage(frame)
