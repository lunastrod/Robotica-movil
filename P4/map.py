#from GUI import GUI
#from HAL import HAL
#from MAP import MAP

import cv2

"""
HAL.setV() - to set the linear speed
HAL.setW() - to set the angular velocity
HAL.getPose3d() - returns x,y and theta components of the robot in world coordinates
GUI.showNumpy(numpy) - shows Gradient Path Planning field on the user interface. It represents the values of the field that have been assigned to the array passed as a parameter. Accepts as input a two-dimensional numpy array whose values can range from 0 to 255 (grayscale). In order to have a grid with the same resolution as the map, the array should be 400x400
GUI.showPath(array) - shows a path on the map. The parameter should be a 2D array containing each of the points of the path
GUI.getTargetPose() - returns x,y coordinates of chosen destionation in the world. Destination is set by clicking on the map image
MAP.getMap() - Returns map image opencv data in opencv data (400x400px)
MAP.rowColumn(vector) - returns the index in map coordinates corresponding to the vector in world coordinates passed as parameter
"""
"""OLD FUNCTIONS"""
def oldwave_expand(map, frontier):
    #This function expands the wavefront.
    # PARAMETERS
    # map - map of the environment
    # frontier - frontier of the wavefront
    # RETURN
    # map - map with the wavefront expanded
    # frontier - new frontier of the wavefront

    #expand at distance 1
    neighbours = [[0,1],[0,-1],[1,0],[-1,0]]
    for i in range(len(neighbours)):
        try:
            point = frontier[0]
            point=[point[0]+neighbours[i][0],point[1]+neighbours[i][1]]
            if(map[point[0]][point[1]]==0):
                frontier.append(point)
                map[point[0]][point[1]]=map[frontier[0][0]][frontier[0][1]]+1
        except:
            pass
    #expand at distance 1.4 (diagonal)
    neighbours = [[1,1],[1,-1],[-1,1],[-1,-1]]
    for i in range(len(neighbours)):
        try:
            point = frontier[0]
            point=[point[0]+neighbours[i][0],point[1]+neighbours[i][1]]
            if(map[point[0]][point[1]]==0):
                frontier.append(point)
                map[point[0]][point[1]]=map[frontier[0][0]][frontier[0][1]]+1.4
        except:
            pass
    try:
        frontier.pop(0)
    except:
        pass
    return map, frontier

def oldwave_path(map, start):
    #This function finds the path from the start to the goal.
    # PARAMETERS
    # map - map of the environment
    # start - start position in the map coordinates
    # RETURN
    # path - path from start to goal

    path=[]
    path.append(start)
    while(map[path[-1][0]][path[-1][1]]!=0):#while not at goal
        try:
            neighbours = [[0,1],[0,-1],[1,0],[-1,0],[1,1],[1,-1],[-1,1],[-1,-1]]
            for i in range(len(neighbours)):
                point = path[-1]
                point=[point[0]+neighbours[i][0],point[1]+neighbours[i][1]]
                if(map[point[0]][point[1]]<map[path[-1][0]][path[-1][1]]):
                    path.append(point)
                    break
        except:
            pass
    return path



def oldwave_front(map, start, goal):
    #This function implements the wavefront algorithm.
    # PARAMETERS
    # map - map of the environment
    # goal - goal position in the map coordinates
    # start - start position in the map coordinates
    # RETURN
    # path - path from start to goal
    # map - map with the path marked

    done=False
    frontier=[]
    frontier.append(goal)
    while(not done):
        map,frontier=wave_expand(map, frontier)
        if(start in frontier or len(frontier)==0):
            done=True
    return map, wave_path(map, start)


"""NEW FUNCTIONS"""

def wave_expand(map, frontier):
    #This function expands the wavefront.
    # PARAMETERS
    # map - map of the environment
    # frontier - frontier of the wavefront
    # RETURN
    # map - map with the wavefront expanded
    # frontier - new frontier of the wavefront

    #expand at distance 1
    x=frontier[0][0]
    y=frontier[0][1]
    neighbours = [[y,x+1],[y,x-1],[y+1,x],[y-1,x]]
    for i in range(len(neighbours)):
        if(neighbours[i][0] in range(len(map[0])-1) and neighbours[i][1] in range(len(map)-1)):#if in map
            if(map[neighbours[i][0]][neighbours[i][1]]==0):#if not obstacle or visited
                frontier.append(neighbours[i])#add to frontier
                map[neighbours[i][0]][neighbours[i][1]]=map[x][y]+1#update map
    #expand at distance 1.4 (diagonal)
    neighbours = [[x+1,y+1],[x+1,y-1],[x-1,y+1],[x-1,y-1]]
    
    for i in range(len(neighbours)):
        if(neighbours[i][0] in range(len(map[0])-1) and neighbours[i][1] in range(len(map)-1)):
            print(len(map[0]),len(map),neighbours[i][0],neighbours[i][1],map)
            if(map[neighbours[i][0]][neighbours[i][1]]==0):
                frontier.append(neighbours[i])
                map[neighbours[i][0]][neighbours[i][1]]=map[x][y]+1.4

    return map, frontier[1:]
        

#img = cv2.imread('cityLargeBin.png', cv2.IMREAD_GRAYSCALE)
map=[[0,0,0,0],[0,0,0,0]]
frontier=[[1,1]]
print(map, frontier)
map,frontier=wave_expand(map, frontier)
print(map, frontier)

#cv2.imshow('image',img)
#cv2.waitKey(0)
#cv2.destroyAllWindows()


