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
    #PARAMETERS
    obstacle_k = 0.01
    goal_k = 4
    ignore_dist = 10#m
    max_goal_force = 5
    max_obstacle_force = 45090000000000


    ignore_dist*=ignore_dist
    obstacle_force = [0, 0]
    goal_force = [0, 0]
    for i in range(len(laser)):
        #if the distance to the obstacle is more than ignore_dist, then the force is 0
        if(abs(laser[i][0])+abs(laser[i][1])>ignore_dist):
            continue
        #the force is inversely proportional to the square of the distance to the obstacle
        #calculate the distance to the obstacle
        sqr_dist=laser[i][0]*laser[i][0]+laser[i][1]*laser[i][1]
        #update the obstacle force
        obstacle_force[0]=obstacle_force[0]-laser[i][0]/sqr_dist*obstacle_k
        obstacle_force[1]=obstacle_force[1]-laser[i][1]/sqr_dist*obstacle_k

    obstacle_force[0]=min(max(obstacle_force[0],-max_obstacle_force),max_obstacle_force)
    obstacle_force[1]=min(max(obstacle_force[1],-max_obstacle_force),max_obstacle_force)

    #update the goal force
    goal_force[0]=min(max_goal_force,goal_force[0]+goal[0]*goal_k)
    goal_force[1]=min(max_goal_force,goal_force[1]+goal[1]*goal_k)
    return obstacle_force,goal_force,tuple(map(sum, zip(obstacle_force, goal_force)))

def force_to_vw(force):
    """
    This function converts the control signal (force) into the desired linear and angular velocities of the robot.
    """
    maxv=0
    maxw=math.pi/2

    angle=math.atan2(force[1],force[0])
    #print("prev angle",math.degrees(angle))
    angle=(angle-math.pi/2)
    if(angle<-math.pi):
        angle=angle+2*math.pi
    print("angle",math.degrees(angle),angle)

    #v=math.cos(angle)*maxv
    v=max(0,-1.62*angle**2+1)*maxv
    #w=math.sin(angle)*maxw
    w=max(-maxw,min(maxw,3.2*angle**3))
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
    robotx=HAL.getPose3d().x
    roboty=HAL.getPose3d().y
    robotyaw=HAL.getPose3d().yaw
    # Get the next waypoint
    currentTarget = GUI.map.getNextTarget()
    #print(dir(currentTarget))
    target=[currentTarget.getPose().x,currentTarget.getPose().y]
    #target=[0,0]
    # Convert the waypoint to relative coordinates
    local_target = absolute2relative(target[0], target[1], robotx, roboty, robotyaw)
    if(is_close(local_target)):
        currentTarget.setReached(True)
        
    print("local target: ", local_target)
    # Parse the laser data
    laser=vectorize_laser(parse_laser_data(HAL.getLaserData()))
    #print(laser)
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
    HAL.setW(robotw)
    HAL.setV(robotv)
    # Update the GUI
    GUI.showForces(obstacle_force, goal_force, force)
    # show image
    GUI.showImage(HAL.getImage())
    # show target
    GUI.showLocalTarget(target)
    
    
    