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
    START_RIGHT = 8
    RIGHT = 9

LINEAR_VELOCITY = 1
ANGULAR_VELOCITY = 1

state = State.START_SPIRAL
state_variable = 0

def read_sensors(laser_data):
    """Read the sensors"""
    laser_left = 0
    laser_right = 0
    laser_fd = 0
    ANGLE_LEFT=30
    ANGLE_RIGHT=150
    for i in range(180):
        laser_left=min(laser_data.values[0:ANGLE_LEFT])
        laser_right=min(laser_data.values[ANGLE_RIGHT:180])
        laser_fd=min(laser_data.values[ANGLE_LEFT:ANGLE_RIGHT])
    return (laser_left, laser_fd, laser_right) #mm

def check_obstacle(laser):
    """Check if there is an obstacle in front of the robot"""
    global state
    if(laser[0] < 0.25 or laser[1] < 0.25 or laser[2] < 0.25):
        state=State.START_BW
        return True
    return False
    
def print_state():
    """Print the state of the robot"""
    global state, state_variable
    if state == State.SPIRAL:
        print("Spiral", state_variable)
    elif state == State.FD:
        print("Forward", state_variable)
    elif state == State.BW:
        print("Backward", state_variable)
    elif state == State.LEFT:
        print("Left", state_variable)
    elif state == State.RIGHT:
        print("Right", state_variable)
    

def change_state(laser):
    """
    Change the state of the robot
    the robot will spiral until it detects an obstacle
    then it will go back and turn
    then it will go forward
    if it doesn't detect an obstacle it will spiral again
    """
    global state, state_variable
    if state == State.START_BW:
        state_variable = random.randint(10, 20) #iterations until turn
        state = State.BW
    elif state == State.START_SPIRAL:
        state_variable = 0 #iterations in spiral
        state = State.SPIRAL
    elif state == State.START_FD:
        state_variable = 20 #iterations until able to spiral
        state = State.FD
    elif state == State.START_LEFT:
        state_variable = random.randint(0, 50) #iterations until forward
        state = State.LEFT
    elif state == State.START_RIGHT:
        state_variable = random.randint(0, 50) #iterations until forward
        state = State.RIGHT
    
    elif state == State.BW and state_variable <= 0: #if backward is finished
        if(laser[0]>laser[2]): #if there is more space on the left
            state = State.START_LEFT 
        else: 
            state = State.START_RIGHT
    elif state == State.LEFT and state_variable <= 0: #if left is finished
        state = State.START_FD 
    elif state == State.RIGHT and state_variable <= 0: #if right is finished
        state = State.START_FD 
    elif state == State.FD and min(laser) > 1 and state_variable <= 0: #if forward is finished and there is no obstacle
        state = State.START_SPIRAL
    


def calculate_VW():
    """Calculate the linear and angular velocity of the robot"""
    global state, state_variable
    if state == State.SPIRAL:
        state_variable += 1 #iterations in spiral
        return (state_variable/300, 1)
    elif state == State.FD:
        state_variable -= 1 #iterations until able to spiral
        return (LINEAR_VELOCITY, 0)
    elif state == State.BW:
        state_variable -= 1 #iterations until turn
        return (-LINEAR_VELOCITY, 0)
    elif state == State.LEFT:
        state_variable -= 1 #iterations until forward
        return (0, -ANGULAR_VELOCITY)
    elif state == State.RIGHT:
        state_variable -= 1 #iterations until forward
        return (0, ANGULAR_VELOCITY)
    return 0, 0


while True:
    time.sleep(1/12)  # 12 Hz
    laser=read_sensors(HAL.getLaserData())
    print(laser)
    if(check_obstacle(laser)):
        print("Obstacle")
    change_state(laser)
    print_state() 

    robot_V, robot_W = calculate_VW()
    HAL.setV(robot_V) 
    HAL.setW(robot_W) 




