import random
import time
from HAL import HAL
from GUI import GUI


class State:
    START_SPIRAL = 0
    SPIRAL = 1
    START_FD = 2
    FD = 3
    START_BW = 4
    BW = 5
    START_LEFT = 6
    LEFT = 7


state = State.START_BW
state_variable = 0

def read_sensors():
    """Read the sensors"""
    HAL.getLaserData()
    laser_left = 0
    laser_right = 0
    laser_fd = 0
    ANGLE_LEFT=30
    ANGLE_RIGHT=150
    for i in range(180):
        laser_left=min(laser.data.values[0:ANGLE_LEFT])
        laser_right=min(laser.data.values[ANGLE_RIGHT:180])
        laser_fd=min(laser.data.values[ANGLE_LEFT:ANGLE_RIGHT])
    return laser_left, laser_fd, laser_right #mm

def check_obstacle():
    """Check if there is an obstacle in front of the robot"""
    global state
    laser_left, laser_fd, laser_right = read_sensors()
    if laser_left < 10 or laser_fd < 10 or laser_right < 10:
        state=State.START_BW
    

def change_state():
    """Change the state of the robot"""
    global state, state_variable
    if state == State.START_BW:
        state_variable = 0
        state = State.BW
    elif state == State.START_SPIRAL:
        state_variable = 0
        state = State.SPIRAL
    elif state == State.START_FD:
        state_variable = 0
        state = State.FD
    elif state == State.START_LEFT:
        state_variable = 0
        state = State.LEFT

    elif state == State.BW and state_variable <= 0:
        state = State.START_LEFT
    elif state == State.LEFT and state_variable <= 0:
        state = State.START_FD


def calculate_VW():
    """Calculate the linear and angular velocity of the robot"""
    global state, state_variable
    if state == State.SPIRAL:
        print("SPIRAL")
        state_variable += 1
        return 0.1, 0.1
    elif state == State.FD:
        print("FD")
        state_variable -= 1
        return 0.1, 0
    elif state == State.BW:
        print("BW")
        state_variable -= 1
        return -0.1, 0
    elif state == State.LEFT:
        print("LEFT")
        state_variable -= 1
        return 0, 0.1


while True:
    time.sleep(1/12)  # 12 Hz
    laser = read_sensors() 

    robot_V, robot_W = calculate_VW()
    HAL.setV(robot_V) 
    HAL.setW(robot_W) 

    check_obstacle() 
    change_state() 


