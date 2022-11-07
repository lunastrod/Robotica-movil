import math

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
    maxv=5
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


"""
for i in range(-90, 270, 10):
    print(force_to_vw((math.cos(math.radians(i)), math.sin(math.radians(i)))))
"""

"""
for l in laser:
    print(("[%0.2f,%0.2f]," % (l[0],l[1])),end="")
"""

#laser=[[1,1],[-1,1]]
print(VFF_controller(laser,[0,5]))
print(force_to_vw([-0.005, -0.015]))