from rrttree import Node
from collections import deque
from queue import PriorityQueue
import heapq
import numpy as np
import math
import random as rd
import pygame
import time


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
    ptInCircle3 = (x - math.floor(5 / r)) ** 2 + (y - math.floor(5 / r)) ** 2.0 - (
            (1.0 + radiusClearance) / r) ** 2 <= 0

    # ------------------------------------------------------------------------------
    #                              Circle 4 pts
    # ------------------------------------------------------------------------------
    ptInCircle4 = (x - math.floor(3 / r)) ** 2 + (y - math.floor(8 / r)) ** 2.0 - (
            (1.0 + radiusClearance) / r) ** 2 <= 0

    # --------------------------------------------------------------------------------
    #                             square 1 pts
    # --------------------------------------------------------------------------------
    X = np.float32([2.25, 3.75, 3.75, 2.25]) / r
    Y = np.float32([1.25, 1.25, 2.75, 2.75]) / r
    ptInRectangle = Y[0] - radiusClearance / r <= y <= Y[2] + radiusClearance / r and \
                    0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / r and \
                    0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / r

    # --------------------------------------------------------------------------------
    #                             Square 2 pts
    # --------------------------------------------------------------------------------
    X = np.float32([0.25, 1.75, 1.75, 0.25]) / r
    Y = np.float32([4.25, 4.25, 5.75, 5.75]) / r
    ptInSquare1 = Y[0] - radiusClearance / r <= y <= Y[2] + radiusClearance / r and \
                  0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / r and \
                  0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / r

    # --------------------------------------------------------------------------------
    #                             Square 3 pts
    # --------------------------------------------------------------------------------
    X = np.float32([8.25, 9.75, 9.75, 8.25]) / r
    Y = np.float32([4.25, 4.25, 5.75, 5.75]) / r
    ptInSquare2 = Y[0] - radiusClearance / r <= y <= Y[2] + radiusClearance / r and \
                  0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / r and \
                  0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / r

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


def pathIsSafe(pt1, pt2, radiusClearance, temp):
    t = np.arange(0.2, 1.2, 0.2)
    v = pt2 - pt1
    for i in range(len(t)):
        r = (t[i] * v + pt1)[0:2]
        if  isSafe(r, 1, radiusClearance):
            if i == 0:
                return [True, pt1]
            else:
                return [True, (t[i - 1] * v + pt1)[0:2]]
    if temp == 1:
        return [True, pt2]
    return [True, r]


# prints solution path
def printPath(node):
    solution = []
    current = node
    while current:
        sol = np.append(current.state)
        solution.append(sol)
        current = current.parent

    return solution


# Normalizing angle and step size
def normalize(coor, threshDistance):
    x, y = coor
    x = round(x / threshDistance) * threshDistance
    y = round(y / threshDistance) * threshDistance
    return [x, y]


# CalcrobotParams[0]ating the Euclidean distance
def distance(startPosition, goalPosition):
    sx, sy = startPosition
    gx, gy = goalPosition
    return math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2)


def generatePoint():
    x = rd.uniform(0.0, 10.0)
    y = rd.uniform(0.0, 10.0)
    return [x, y]


def minimumDistance(nodesExplored, newState):
    mininum = 100
    st = ""
    for key, node in nodesExplored.items():
        dist = distance(node.state, newState)
        if dist < mininum:
            mininum = dist
            st = key
    return st, mininum


# generates optimal path for robot
def generatePath(q, startEndCoor, nodesExplored, dt, radiusClearance, threshDistance=0.1, threshAngle=5):
    # normalize goal and start positions
    sx, sy = normalize(startEndCoor[0], threshDistance)
    gx, gy = normalize(startEndCoor[1], threshDistance)

    # Initializing root node
    key = str(sx) + str(sy)
    root = Node(np.float32([sx, sy]), 0.0, None)
    nodesExplored[key] = root

    count = 1
    # heapq.heappush(q, (root.cost, count, root))

    while True:#len(q) > 0:
        # print(len(q))
        # _, _, currentNode = heapq.heappop(q)
        if count == 1:
            currentNode = root
        # print(distance(currentNode.state, [gx, gy]))
        if distance(currentNode.state, [gx, gy]) <= 0.3:
            sol = printPath(currentNode)
            return [True, sol]

            # for actions in range(8):
            #     x, y, t = currentNode.state
            #
            #     # Defining actions based on constraints
            #     if actions == 0:
            #         newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, 0, robotParams[0],
            #                                                                             robotParams, dt)
            #     elif actions == 1:
            #         newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, robotParams[0], 0,
            #                                                                             robotParams, dt)
            #     elif actions == 2:
            #         newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, robotParams[0],
            #                                                                             robotParams[0], robotParams, dt)
            #     elif actions == 3:
            #         newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, 0, robotParams[1],
            #                                                                             robotParams, dt)
            #     elif actions == 4:
            #         newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, robotParams[1], 0,
            #                                                                             robotParams, dt)
            #     elif actions == 5:
            #         newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, robotParams[1],
            #                                                                             robotParams[1], robotParams, dt)
            #     elif actions == 6:
            #         newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, robotParams[0],
            #                                                                             robotParams[1], robotParams, dt)
            #     elif actions == 7:
            #         newPosX, newPosY, newOrientation, x_dot, y_dot, omega = constraints(x, y, t, robotParams[1],
            #                                                                             robotParams[0], robotParams, dt)

        newPosX, newPosY = generatePoint()
        # print(newPosX, newPosY)
        newState = np.array([newPosX, newPosY])# np.array(normalize([newPosX, newPosY], threshDistance))
        s = str(newState[0]) + str(newState[1])
        parentkey, dist = minimumDistance(nodesExplored, newState)

        if s not in nodesExplored:
            if isSafe(newState, 1, radiusClearance):
                status = False
                while not status:
                    status, newState = pathIsSafe(newState, nodesExplored[parentkey].state, radiusClearance, 1)
                    # print(status)
            else:
                status, newState = pathIsSafe(newState, nodesExplored[parentkey].state, radiusClearance, 0)
                print(status)
            # newCostToCome = currentNode.costToCome + distance(newState, currentNode.state)
            newCost = nodesExplored[parentkey].cost + distance(newState,
                                                  nodesExplored[parentkey].state)  # distance(newState, [gx, gy, gz])

            newNode = Node(newState, newCost, nodesExplored[parentkey])
            # newNode.velocities = [x_dot, y_dot, omega]
            nodesExplored[s] = newNode

            # heapq.heappush(q, (newNode.cost, count, newNode))
            count += 1
            currentNode = newNode
            print(count)

        else:
            # if (nodesExplored[s].cost > currentNode.costToCome + distance(newState, currentNode.state) + distance(
            #         newState, [gx, gy, gt])):
            #     nodesExplored[s].costToCome = currentNode.costToCome + distance(newState, currentNode.state)
            #     nodesExplored[s].cost = nodesExplored[s].costToCome + distance(newState, [gx, gy, gt])
            #     nodesExplored[s].parent = currentNode
            #     nodesExplored[s].velocities = [x_dot, y_dot, omega]
            if nodesExplored[s].cost > nodesExplored[parentkey].cost + distance(newState, nodesExplored[parentkey].state):
                nodesExplored[s].cost = nodesExplored[parentkey].cost + distance(newState, nodesExplored[parentkey].state)
                nodesExplored[s].parent = nodesExplored[parentkey]

        if nodesExplored[s].parent:
            pt = nodesExplored[s].state[0:2]
            ptParent = nodesExplored[s].parent.state[0:2]
            x, y = pt * scale * res
            x2, y2 = ptParent * scale * res

            # draw explored nodes
            pygame.draw.line(gameDisplay, white, (x2, y2), (x, y), 1)
            # pygame.draw.circle(gameDisplay,green,(int(x),int(y)),4)
            triangle = triangleCoordinates([x2, y2], [x, y], 5)
            pygame.draw.polygon(gameDisplay, green,
                                [tuple(triangle[0]), tuple(triangle[1]), tuple(triangle[2])])

        # draw start and goal locations
        pygame.draw.rect(gameDisplay, blue, (startCoor[0] * res * scale, startCoor[1] * res * scale, res * 2, res * 2))

        pygame.draw.circle(gameDisplay, blue,
                           (int(goalCoor[0] * res * scale), int(goalCoor[1] * res * scale)),
                           math.floor(0.3 * res * scale))

        pygame.draw.rect(gameDisplay, white, (goalCoor[0] * res * scale, goalCoor[1] * res * scale,
                                              res * 2, res * 2))
        pygame.display.update()

    return [False, None]


# Functions which defines actions considering the constraints
def constraints(X0, Y0, Theta0, UL, UR, robotParams, dt):
    # r = robotParams[2]  # Radius of the wheel
    # L = robotParams[3]  # Distance between the wheels
    # smoothCoef = robotParams[4]
    #
    # x_dot = r / 2 * (UL + UR) * math.cos(math.radians(Theta0))
    # y_dot = r / 2 * (UL + UR) * math.sin(math.radians(Theta0))
    # omega = smoothCoef * (r / L) * (UR - UL)  # The smoothing coefficient limits sharp turns
    # # smaller value results in smaller angle changes
    # dx = x_dot * dt
    # dy = y_dot * dt
    # dtheta = omega * dt
    #
    # Xn = X0 + dx
    # Yn = Y0 + dy
    # Thetan = (Theta0 + math.degrees(dtheta)) % 360
    #
    # return Xn, Yn, Thetan, x_dot, y_dot, omega
    pass


def triangleCoordinates(start, end, triangleSize=5):
    rotation = (math.atan2(start[1] - end[1], end[0] - start[0])) + math.pi / 2
    # print(math.atan2(start[1] - end[1], end[0] - start[0]))
    rad = math.pi / 180

    coordinateList = np.array([[end[0], end[1]],
                               [end[0] + triangleSize * math.sin(rotation - 165 * rad),
                                end[1] + triangleSize * math.cos(rotation - 165 * rad)],
                               [end[0] + triangleSize * math.sin(rotation + 165 * rad),
                                end[1] + triangleSize * math.cos(rotation + 165 * rad)]])

    return coordinateList


if __name__ == "__main__":

    # iul = 20
    # iur = 20
    is1 = 0  # -4  #-4
    is2 = 3  # -4  #-3
    ig1 = 0  # 4   #0
    ig2 = 2  # 2.5  #-3
    # istartOrientation = 0
    # idt = -1#0.6 #0.8
    # ismoothCoef = -1# 0.2 #0.1

    # ---------------------------------
    # Inputs From World Coordinates
    # To Pygame Coordinates
    # ---------------------------------
    # startOrientation = 360 - istartOrientation
    # ul = iul
    # ur = iur
    s1 = 5 + (is1)
    s2 = 5 - (is2)
    g1 = 5 + (ig1)
    g2 = 5 - (ig2)
    # dt = idt if (idt >= 0.0) else 0.3
    # smoothCoef = ismoothCoef if (ismoothCoef >= 0) else 0.5

    # ---------------------------
    #  Precision Parameters
    # ---------------------------
    threshDistance = 0.5
    clearance = 0.3
    # threshAngle = 5

    # ---------------------------
    #  Robot parameters
    # ---------------------------
    # smoothCoef = ismoothCoef if (ismoothCoef>= 0) else 0.5
    # wheelDist = 0.2116  # 0.3175/6 * 4
    # wheelRadius = 0.038
    # robotParams = [ul, ur, wheelRadius, wheelDist, smoothCoef]
    robotRadius = 0  # 0.177

    # -------------------------------
    #  Parameters needed by gazebo
    # -------------------------------
    # dt - affects publishing rate
    #   - dt must be of resolution 0.1
    #   - restricting frequency to 10Hz in gazebo
    #   - 1/frequency*dt must be a whole number

    # is1,is2,iorientation- initial pose for robot
    # writeParametersForGazebo(dt, is1, is2, istartOrientation)

    # ----------------------------
    #  Display parameters
    # ----------------------------
    pygame.init()

    res = 1.0  # resolution of grid
    scale = 80  # scale of grid

    white = (255, 255, 255)
    black = (0, 0, 0)
    red = (255, 0, 0)
    green = (0, 255, 0)
    blue = (0, 0, 255)
    yellow = (255, 255, 0)

    size_x = 10
    size_y = 10
    gameDisplay = pygame.display.set_mode((size_x * scale, size_y * scale))

    # ----------------------------
    # Start and goal coordinates
    # ----------------------------
    startCoor = np.float32((np.float32([s1, s2])) / res)
    goalCoor = np.float32((np.float32([g1, g2])) / res)

    startEndCoor = [startCoor, goalCoor]

    ############################################################
    #                 Display Obstacles
    ############################################################
    circlePts1 = [7, 2, 1]
    circlePts2 = [7, 8, 1]
    circlePts3 = [5, 5, 1]
    circlePts4 = [3, 8, 1]

    pygame.draw.circle(gameDisplay, red, (circlePts1[0] * scale, circlePts1[1] * scale), circlePts1[2] * scale)
    pygame.draw.circle(gameDisplay, red, (circlePts2[0] * scale, circlePts2[1] * scale), circlePts2[2] * scale)
    pygame.draw.circle(gameDisplay, red, (circlePts3[0] * scale, circlePts3[1] * scale), circlePts3[2] * scale)
    pygame.draw.circle(gameDisplay, red, (circlePts4[0] * scale, circlePts4[1] * scale), circlePts4[2] * scale)
    pygame.draw.rect(gameDisplay, red, [int(scale * 2.25), int(scale * 1.25), int(scale * 1.5), int(scale * 1.5)])
    pygame.draw.rect(gameDisplay, red, [int(scale * 0.25), int(scale * 4.25), int(scale * 1.5), int(scale * 1.5)])
    pygame.draw.rect(gameDisplay, red, [int(scale * 8.25), int(scale * 4.25), int(scale * 1.5), int(scale * 1.5)])

    ############################################################
    #          Draw Explored Nodes and solution path
    ############################################################
    nodesExplored = {}
    q = []

    if not isSafe(startCoor, res, clearance + robotRadius) or not isSafe(goalCoor, res, clearance + robotRadius):
        pygame.draw.rect(gameDisplay, blue, (startCoor[0] * res * scale, startCoor[1] * res * scale,
                                             res * 2, res * 2))

        pygame.draw.circle(gameDisplay, blue, (int(goalCoor[0] * res * scale), int(goalCoor[1] * res * scale)),
                           math.floor(0.3 * res * scale))

        pygame.draw.rect(gameDisplay, white, (goalCoor[0] * res * scale, goalCoor[1] * res * scale,
                                              res * 2, res * 2))
        basicfont = pygame.font.SysFont(None, 48)
        text = basicfont.render('Start or goal position must be in a valid workspace', True, (255, 0, 0),
                                (255, 255, 255))
        textrect = text.get_rect()
        textrect.centerx = gameDisplay.get_rect().centerx
        textrect.centery = gameDisplay.get_rect().centery

        gameDisplay.blit(text, textrect)
        pygame.display.update()
        pygame.time.delay(2000)

    else:
        startTime = time.time()  # Start time of simulation
        print('Exploring nodes...')
        success, solution = generatePath(q, startEndCoor, nodesExplored, clearance + robotRadius,
                                         threshDistance)
        # print(success)
        endTime = time.time()

        #############################################
        #      Drawing
        #############################################
        if success:
            print('Optimal path found')
            print("Total time taken for exploring nodes " + str(endTime - startTime) + " seconds.")
            # writeSolutionToFile(solution)

            draw = True
            while draw:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        quit()

                # draw nodesExplored
                for s in nodesExplored:
                    if nodesExplored[s].parent:
                        pt = nodesExplored[s].state[0:2]
                        ptParent = nodesExplored[s].parent.state[0:2]
                        x, y = pt * scale * res
                        x2, y2 = ptParent * scale * res

                        # draw explored nodes
                        pygame.draw.line(gameDisplay, white, (x2, y2), (x, y), 1)
                        # pygame.draw.circle(gameDisplay,green,(int(x),int(y)),4)
                        triangle = triangleCoordinates([x2, y2], [x, y], 5)
                        pygame.draw.polygon(gameDisplay, green,
                                            [tuple(triangle[0]), tuple(triangle[1]), tuple(triangle[2])])

                    # draw start and goal locations
                    pygame.draw.rect(gameDisplay, blue, (startCoor[0] * res * scale, startCoor[1] * res * scale, \
                                                         res * 2, res * 2))

                    pygame.draw.circle(gameDisplay, blue,
                                       (int(goalCoor[0] * res * scale), int(goalCoor[1] * res * scale)), \
                                       math.floor(0.3 * res * scale))

                    pygame.draw.rect(gameDisplay, white, (goalCoor[0] * res * scale, goalCoor[1] * res * scale, \
                                                          res * 2, res * 2))
                    pygame.display.update()

                # draw solution path
                for i in range(len(solution) - 2, -1, -1):
                    pt = solution[i][0:2]
                    pt1 = solution[i + 1][0:2]
                    xt, yt = pt[0] * scale * res, pt[1] * scale * res
                    x, y = pt1[0] * scale * res, pt1[1] * scale * res
                    pygame.draw.line(gameDisplay, yellow, (xt, yt), (x, y), 3)
                    pygame.draw.circle(gameDisplay, red, (int(x), int(y)), 4)
                    pygame.display.update()
                pygame.time.delay(4000)
                draw = False

        else:
            print('Path not possible')
            basicfont = pygame.font.SysFont(None, 48)
            text = basicfont.render('Path can\'t be generated', True, (255, 0, 0), (255, 255, 255))
            textrect = text.get_rect()
            textrect.centerx = gameDisplay.get_rect().centerx
            textrect.centery = gameDisplay.get_rect().centery

            gameDisplay.blit(text, textrect)
            pygame.display.update()
            pygame.time.delay(2000)

    pygame.quit()
