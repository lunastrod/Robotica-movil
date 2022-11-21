from GUI import GUI
from HAL import HAL
import math
import numpy as np


def parse_laser_data(laser_data):
    laser = []
    i = 0
    while (i < 180):
        dist = laser_data.values[i]
        if dist > 10:
            dist = 10
        # because the front of the robot is -90 degrees
        angle = math.radians(i-90)
        laser += [(dist, angle)]
        i += 1
    return laser


def vectorize_laser(laser):
    laser_vectorized = []
    for d, a in laser:
        # (4.2.1) laser into GUI reference system
        x = d * math.cos(a) * -1
        y = d * math.sin(a) * -1
        v = (x, y)
        laser_vectorized += [v]
    return laser_vectorized


def VFF_controller(laser, goal):
    """
    This function implements the Virtual Force Field controller.
    Each object in the environment generates a repulsive force towards the robot.
    Goal generates an attractive force in the robot.
    The sum of all forces is the control signal.
    """
    # PARAMETERS
    obstacle_k = 0.04
    goal_k = 2
    max_goal_force = 2.6

    # generate weights for each direction
    weights = [0]*len(laser)
    for i in range(len(weights)):
        if(i > len(laser)*0.4 and i < len(laser)*0.6):
            weights[i] = 1
        elif(i > len(laser)*0.2 and i < len(laser)*0.8):
            weights[i] = 0.2
        else:
            weights[i] = 1.2

    obstacle_force = [0, 0]
    goal_force = [0, 0]
    for i in range(len(laser)):
        # the force is inversely proportional to the square of the distance to the obstacle
        # calculate the distance to the obstacle
        sqr_dist = laser[i][0]*laser[i][0]+laser[i][1]*laser[i][1]
        # update the obstacle force
        obstacle_force[0] = obstacle_force[0] + laser[i][0]/sqr_dist*obstacle_k*weights[i]
        obstacle_force[1] = obstacle_force[1] + laser[i][1]/sqr_dist*obstacle_k*weights[i]


    # update the goal force
    goal_mod = math.sqrt(goal[0]*goal[0]+goal[1]*goal[1])
    unit_goal = [goal[0]/goal_mod, goal[1]/goal_mod]
    goal_strength = min(max_goal_force, goal_k*goal_mod)
    goal_force = [unit_goal[0]*goal_strength, unit_goal[1]*goal_strength]
    return obstacle_force, goal_force, tuple(map(sum, zip(obstacle_force, goal_force)))


def force_to_vw(force):
    """
    This function converts the control signal (force) into the desired linear and angular velocities of the robot.
    """
    maxv = 5
    maxw = 0.8
    kv = 2
    kw = 10

    angle = math.atan2(force[1], force[0])
    #print("prev angle",math.degrees(angle))
    print("angle", math.degrees(angle))
    # v=math.cos(angle)*maxv
    # v=max(0,-1.62*angle**2+1)*maxv
    v = maxv/(1+kv*angle**2)-0.3
    # w=math.sin(angle)*maxw
    w = max(-maxw, min(maxw, kw*angle**3))
    print("force", force, "translates to v,w", v, w)
    return v, w


def absolute2relative(x_abs, y_abs, robotx, roboty, robott):
    """
    We have 2 different coordinate systems in this exercise.

    Absolute coordinate system: Its origin (0,0) is located in the finish line of the circuit (exactly where the F1 starts the lap).
    Relative coordinate system: It is the coordinate system solidary to the robot (F1). Positive values of X means ‘forward’, and positive values of Y means ‘left’.
    You can use the following code to convert absolute coordinates to relative ones (solidary to the F1).
    """

    # robotx, roboty are the absolute coordinates of the robot
    # robott is its absolute orientation
    # Convert to relatives
    dx = x_abs - robotx
    dy = y_abs - roboty

    # Rotate with current angle
    x_rel = dx * math.cos(-robott) - dy * math.sin(-robott)
    y_rel = dx * math.sin(-robott) + dy * math.cos(-robott)

    return x_rel, y_rel


def is_close(goal):
    """
    This function checks if the robot is close enough to the goal.
    """
    threshold = 1
    if(abs(goal[0]) < threshold and abs(goal[1]) < threshold):
        return True
    return False


while True:
    # Get the current position of the robot
    robotx = HAL.getPose3d().x
    roboty = HAL.getPose3d().y
    robotyaw = HAL.getPose3d().yaw
    # Get the next waypoint
    currentTarget = GUI.map.getNextTarget()
    # print(dir(currentTarget))
    target = [currentTarget.getPose().x, currentTarget.getPose().y]
    # target=[0,0]
    # Convert the waypoint to relative coordinates
    local_target = absolute2relative(
        target[0], target[1], robotx, roboty, robotyaw)
    if(is_close(local_target)):
        currentTarget.setReached(True)

    # Parse the laser data
    laser = vectorize_laser(parse_laser_data(HAL.getLaserData()))
    # print(laser)
    # Calculate the control signal
    obstacle_force, goal_force, force = VFF_controller(laser, local_target)
    robotv, robotw = force_to_vw(force)
    # Send the velocity to the robot
    HAL.setW(robotw)
    HAL.setV(robotv)
    # Update the GUI
    GUI.showForces(goal_force, obstacle_force, force)  # green, red, black
    # show image
    GUI.showImage(HAL.getImage())
    # show target
    GUI.showLocalTarget(target)