from GUI import GUI
from HAL import HAL
import math
import numpy as np

def parse_laser_data (laser_data):
    laser = []
    i = 0
    while (i < 180):
        dist = laser_data.values[i]
        if dist > 10:
            dist = 10
        angle = math.radians(i-90) # because the front of the robot is -90 degrees
        laser += [(dist, angle)]
        i+=1
    return laser

def vectorize_laser(laser):
    laser_vectorized = []
    for d, a in laser:
        # (4.2.1) laser into GUI reference system
        x = d * math.cos (a) * -1
        y = d * math.sin (a) * -1
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
    
    repulsive_k = -0.5
    attractive_k = 1
    ignore_dist = 0.5#m
    ignore_dist*=ignore_dist
    obstacle_force = [0, 0]
    goal_force = [0, 0]
    for i in range(len(laser)):
        #if the distance to the obstacle is more than ignore_dist, then the force is 0
        if(abs(laser[i][0])+abs(laser[i][1])>ignore_dist):
            continue
        obstacle_force[0]=obstacle_force[0]+repulsive_k*laser[i][0]
        obstacle_force[1]=obstacle_force[1]+repulsive_k*laser[i][1]
    goal_force[0]=goal_force[0]+attractive_k*goal[0]
    goal_force[1]=goal_force[1]+attractive_k*goal[1]
    return obstacle_force,goal_force,tuple(map(sum, zip(obstacle_force, goal_force)))

def force_to_vw(force):
    """
    This function converts the control signal (force) into the desired linear and angular velocities of the robot.
    """
    angle=math.atan2(force[1],force[0])
    maxv=0.5
    maxw=math.pi/2
    v=math.cos(angle)*maxv
    w=math.sin(angle)*maxw
    return v, w

def absolute2relative (x_abs, y_abs, robotx, roboty, robott):
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
    x_rel = dx * math.cos (-robott) - dy * math.sin (-robott)
    y_rel = dx * math.sin (-robott) + dy * math.cos (-robott)

    return x_rel, y_rel

while True:
    # Get the current position of the robot
    robotx=HAL.getPose3d().x
    roboty=HAL.getPose3d().y
    robotyaw=HAL.getPose3d().yaw
    # Get the next waypoint
    target = [1.0,1.0]#global coords
    # Convert the waypoint to relative coordinates
    local_target = absolute2relative(target[0], target[1], robotx, roboty, robotyaw)
    # Parse the laser data
    laser=vectorize_laser(parse_laser_data(HAL.getLaserData()))
    print(laser)
    # Calculate the control signal
    obstacle_force,goal_force,force=VFF_controller(laser,local_target)
    print("obstacle_force: ",obstacle_force)
    print("goal_force: ",goal_force)
    print("force: ",force)
    robotv, robotw = force_to_vw(force)
    print("v: ", robotv)
    print("w: ", robotw)
    print()
    # Send the velocity to the robot
    #HAL.setW(robotw)
    #HAL.setV(robotv)
    # Update the GUI
    GUI.showForces(obstacle_force, goal_force, force)
    # show image
    GUI.showImage(HAL.getImage())
    # show target
    GUI.showLocalTarget(target)
    