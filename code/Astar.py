from namenode import Node
from collections import deque
from queue import PriorityQueue
import heapq
import numpy as np
import math


######################################
#          Workspace
######################################
def isValidWorkspace(pt, r, radiusClearance):  
    x, y = pt

    # ------------------------------------------------------------------------------
    #                              Circle 1 pts
    # ------------------------------------------------------------------------------
    ptInCircle1 = (x - math.floor(7 / r)) ** 2 + (y - math.floor(2 / r)) ** 2 - ((1 + radiusClearance) / r) ** 2 <= 0

    # ------------------------------------------------------------------------------
    #                              Circle 2 pts
    # ------------------------------------------------------------------------------
    ptInCircle2 = (x - math.floor(7 / r)) ** 2 + (y - math.floor(8 / r)) ** 2 - ((1 + radiusClearance) / r) ** 2 <= 0

    # ------------------------------------------------------------------------------
    #                              Circle 3 pts
    # ------------------------------------------------------------------------------
    ptInCircle3 = (x - math.floor(5 / r)) ** 2 + (y - math.floor(5 / r)) ** 2.0 - ((1.0 + radiusClearance) / r) ** 2 <= 0

    # ------------------------------------------------------------------------------
    #                              Circle 4 pts
    # ------------------------------------------------------------------------------
    ptInCircle4 = (x - math.floor(3 / r)) ** 2 + (y - math.floor(8 / r)) ** 2.0 - ((1.0 + radiusClearance) / r) ** 2 <= 0


    # --------------------------------------------------------------------------------
    #                             square 1 pts
    # --------------------------------------------------------------------------------
    X = np.float32([2.25, 3.75, 3.75, 2.25]) / r
    Y = np.float32([1.25, 1.25, 2.75, 2.75]) / r
    ptInRectangle = y >= Y[0] - radiusClearance / r                        and \
                    0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / r  and \
                    y <= Y[2]+ radiusClearance / r                         and \
                    0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / r 

    # --------------------------------------------------------------------------------
    #                             Square 2 pts
    # --------------------------------------------------------------------------------
    X = np.float32([0.25, 1.75, 1.75, 0.25])/r
    Y = np.float32([4.25, 4.25, 5.75, 5.75])/r
    ptInSquare1 = y  >= Y[0] - radiusClearance / r                        and \
                  0  >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / r  and \
                  y  <= Y[2]+ radiusClearance / r                         and \
                  0  >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / r 


    # --------------------------------------------------------------------------------
    #                             Square 3 pts
    # --------------------------------------------------------------------------------
    X = np.float32([8.25, 9.75, 9.75, 8.25])/r
    Y = np.float32([4.25, 4.25, 5.75, 5.75])/r
    ptInSquare2 = y  >= Y[0] - radiusClearance / r                        and \
                  0  >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / r  and \
                  y  <= Y[2]+ radiusClearance / r                         and \
                  0  >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / r 

    if ptInCircle1 or ptInCircle2 or ptInCircle3 or ptInCircle4 or ptInRectangle or ptInSquare1 or ptInSquare2:
        return False
    return True


# checks whether next action is near an obstacle or ill defined
def isSafe(newState, r, radiusClearance):
    col = float(10 / r)
    row = float(10 / r)

    if newState[0] < 0.0 or newState[0] > col or newState[1] < 0.0 or newState[1] > row:
        return False
    return isValidWorkspace(newState[0:2], r, radiusClearance)


def pathIsSafe(pt1,pt2,radiusClearance):
    t = np.arange(0.3, 1.0, 0.3)
    v = pt2 - pt1
    for i in range(len(t)):
        r = (t[i]*v + pt1)[0:2]
        if( not isSafe(r,1,radiusClearance)):
            return False
    return True


# prints solution path
def printPath(node):
    solution = []
    current = node
    while (current):
        sol = np.append(current.state, current.velocities)
        solution.append(sol)
        current = current.parent

    return solution 


# Normalizing angle and step size 
def normalize(coor,threshDistance ,threshAngle):
    x, y,t = coor
    x = round(x / threshDistance) * threshDistance
    y = round(y / threshDistance) * threshDistance
    t = round(t / threshAngle) * threshAngle
    return [x, y, t]


# CalcrobotParams[0]ating the Euclidean distance
def distance(startPosition, goalPosition):
    sx, sy,_ = startPosition
    gx, gy,_ = goalPosition
    return math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2)


# generates optimal path for robot
def generatePath(q, startEndCoor, nodesExplored,robotParams,dt,radiusClearance,threshDistance = 0.1,threshAngle = 5):

    # normalize goal and start positions
    sx, sy, st = normalize(startEndCoor[0],threshDistance,threshAngle)
    gx, gy, gt = normalize(startEndCoor[1],threshDistance,threshAngle)

    # Initializing root node
    key = str(sx) + str(sy) + str(st)
    root = Node(np.float32([sx, sy, st]), 0.0, 0.0, None)
    nodesExplored[key] = root

    count = 1
    heapq.heappush(q, (root.cost, count, root))

    while (len(q) > 0):
        _, _, currentNode = heapq.heappop(q)

        if (distance(currentNode.state, [gx,gy,gt]) <= 0.3):
            sol = printPath(currentNode)
            return [True, sol]


        for actions in range(8):
            x, y, t = currentNode.state
            
            # Defining actions based on constraints
            if actions == 0:
                newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, 0, robotParams[0],robotParams, dt)
            elif actions == 1:
                newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, robotParams[0], 0,robotParams,dt)
            elif actions == 2:
                newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, robotParams[0], robotParams[0],robotParams,dt)
            elif actions == 3:
                newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, 0, robotParams[1],robotParams,dt)
            elif actions == 4:
                newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, robotParams[1], 0, robotParams,dt)
            elif actions == 5:
                newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, robotParams[1], robotParams[1], robotParams,dt)
            elif actions == 6:
                newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, robotParams[0], robotParams[1], robotParams,dt)
            elif actions == 7:
                newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, robotParams[1], robotParams[0], robotParams,dt)
           
 
            newState = np.array(normalize([newPosX,newPosY,newOrientation],threshDistance,threshAngle))            
            s = str(newState[0]) + str(newState[1]) + str(newState[2])

            if (s not in nodesExplored):
                if (isSafe(newState, 1, radiusClearance) and pathIsSafe(newState,currentNode.state,radiusClearance)):
                    newCostToCome = currentNode.costToCome + distance(newState,currentNode.state) 
                    newCost = newCostToCome + distance(newState, [gx, gy,gt])
                    
                    newNode = Node(newState, newCost, newCostToCome, currentNode)
                    newNode.velocities = [x_dot, y_dot, omega]
                    nodesExplored[s] = newNode
                    
                    heapq.heappush(q, (newNode.cost, count, newNode))
                    count += 1
            else:
                if (nodesExplored[s].cost > currentNode.costToCome + distance(newState,currentNode.state)+ distance(newState, [gx, gy,gt])):
                    nodesExplored[s].costToCome = currentNode.costToCome + distance(newState,currentNode.state) 
                    nodesExplored[s].cost = nodesExplored[s].costToCome + distance(newState, [gx, gy,gt])
                    nodesExplored[s].parent = currentNode
                    nodesExplored[s].velocities = [x_dot, y_dot, omega]

    return [False, None]


# Functions which defines actions considering the constraints
def constraints(X0, Y0,Theta0,UL,UR,robotParams,dt):
    r = robotParams[2]        # Radius of the wheel  
    L = robotParams[3]        # Distance between the wheels  
    smoothCoef = robotParams[4]
    
    x_dot = r/2 * (UL + UR) * math.cos(math.radians(Theta0)) 
    y_dot = r/2 * (UL + UR) * math.sin(math.radians(Theta0)) 
    omega = smoothCoef*(r / L) * (UR - UL)  # The smoothing coefficient limits sharp turns
                                            # smaller value results in smaller angle changes
    dx = x_dot*dt  
    dy = y_dot*dt 
    dtheta = omega*dt

    Xn = X0 + dx
    Yn = Y0 + dy
    Thetan = (Theta0 +  math.degrees(dtheta))%360

    return Xn, Yn, Thetan, x_dot, y_dot, omega


if __name__ == "__main__":
    pass
    # startOrientation = 360 - 15
    # clearance = 0.1
    # q = []
    # ul = 2
    # ur = 2
    # s1 = 5+(-4)
    # s2 = 5-(4)
    # g1 = 5+(-5)
    # g2 = 5-(5)
    # nodesExplored = {}
    # res = 1

    # startPosition = np.float32((np.float32([s1,s2]))/res)
    # goalPosition = np.float32((np.float32([g1,g2]))/res)

    # generatePath(q,startPosition,startOrientation,goalPosition,nodesExplored,ul, ur,clearance+0.038) 





