import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, PathPatch, Rectangle
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import mpl_toolkits.mplot3d.art3d as art3d
import random as rd


class Node:
    def __init__(self, state=None, parent=None):
        self.state = state
        self.parent = parent


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)


######################################
#          Workspace
######################################
def isValidWorkspace(pt, r, radiusClearance):
    x, y, z = pt

    # ------------------------------------------------------------------------------
    #                              Circle 1 pts
    # ------------------------------------------------------------------------------
    ptInCircle1 = ((3 + radiusClearance) / r) ** 2 >= (x - 5.5) ** 2 + (y - 5.5) ** 2 or \
                  (x - 5.5) ** 2 + (y - 5.5) ** 2 >= ((4.9 - radiusClearance) / r) ** 2

    pointZ = z < 0 or z > 5

    if ptInCircle1 or pointZ:
        return False
    return True


# checks whether next action is near an obstacle or ill defined
def isSafe(newState, r, radiusClearance):
    col = float(10 / r)
    row = float(10 / r)

    if newState[0] < 0.0 or newState[0] > col or newState[1] < 0.0 or newState[1] > row:
        return False
    return isValidWorkspace(newState, r, radiusClearance)


def steer(xNearest, xRand):
    stepsize = 0.8
    dist = distance(xNearest, xRand)
    if (dist < stepsize):
        return xRand
    else:
        t = stepsize / dist
        v = xRand - xNearest
        r = t * v + xNearest
        return r


def isObstacleFree(pt1, pt2, radiusClearance):
    stepsize = 0.1
    t = np.arange(stepsize, 1.0 + stepsize, stepsize)
    v = pt2 - pt1
    for i in range(len(t)):
        r = t[i] * v + pt1
        if not isSafe(r, 1, radiusClearance):
            return False
    return True


def printPath(node):
    solution = []
    current = node
    while current:
        solution.append(current.state)
        current = current.parent
    return solution


def samplePoint():
    x = rd.uniform(0.0, 10.0)
    y = rd.uniform(0.0, 10.0)
    z = rd.uniform(0.0, 5.0)
    return [x, y, z]


def distance(startPosition, goalPosition):
    sx, sy, sz = startPosition
    gx, gy, gz = goalPosition
    return math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2 + (gz - sz) ** 2)


def nearest(nodesExplored, newState):
    minDist = np.inf
    for key, node in nodesExplored.items():
        dist = distance(node.state, newState)
        if dist < minDist:
            minDist = dist
            minKey = key
    return minKey, minDist


def generatePath(q, startEndCoor, nodesExplored, radiusClearance):
    # get start and goal locations
    sx, sy, sz = startEndCoor[0]
    gx, gy, gz = startEndCoor[1]

    # Initializing root node
    key = str(sx) + str(sy) + str(sz)
    root = Node(np.float32([sx, sy, sz]), None)
    nodesExplored[key] = root

    # for i in range(numIterations):
    while True:
        # sample random point
        newPosX, newPosY, newPosZ = samplePoint()
        xRand = np.array([newPosX, newPosY, newPosZ])

        # Get Nearest Node
        xNearestKey, _ = nearest(nodesExplored, xRand)
        xNearest = nodesExplored[xNearestKey].state

        # steer in direction of path
        xNew = steer(xNearest, xRand)

        # check if edge is not in obstacle
        if (xNew == xNearest).all() or not isObstacleFree(xNearest, xNew, radiusClearance):
            continue

        # add node to nodesExplored(add vertex and edge)
        newNode = Node(xNew, nodesExplored[xNearestKey])
        s = str(newNode.state[0]) + str(newNode.state[1]) + str(newNode.state[2])
        nodesExplored[s] = newNode

        # print path if goal is reached
        if distance(newNode.state, [gx, gy, gz]) <= 0.5:
            sol = printPath(newNode)
            return [True, sol]

    return [False, None]


def drawEnv(size_x,size_y,size_z,ax):
    c2 = Circle((size_x / 2, size_y / 2), 3, fill=True, linestyle='-', linewidth=5, color='black', alpha=0.2)
    c3 = Circle((size_x / 2, size_y / 2), 5, fill=True, linestyle='-', linewidth=5, fc='black', ec='black', alpha=0.2)
    c1 = Circle((size_x / 2, size_y / 2), 4, fill=False, linestyle='--', linewidth=5, color='white')
    c2Temp = Circle((size_x / 2, size_y / 2), 3, fill=False, linestyle='-', linewidth=5, color='black')
    c3Temp = Circle((size_x / 2, size_y / 2), 5, fill=False, linestyle='-', linewidth=5, color='black')

    ax.add_patch(c1)
    ax.add_patch(c2)
    ax.add_patch(c3)
    ax.add_patch(c2Temp)
    ax.add_patch(c3Temp)
    art3d.pathpatch_2d_to_3d(c1, z=0, zdir="z")
    art3d.pathpatch_2d_to_3d(c2, z=0, zdir="z")
    art3d.pathpatch_2d_to_3d(c3, z=0, zdir="z")
    art3d.pathpatch_2d_to_3d(c2Temp, z=0, zdir="z")
    art3d.pathpatch_2d_to_3d(c3Temp, z=0, zdir="z")

    r1Temp = Rectangle((0.5, 0), height=size_z, width=2, fill=True, linestyle='-', linewidth=5, color='black',
                       alpha=0.2)
    r1 = Rectangle((0.5, 0), height=size_z, width=2, fill=False, linestyle='-', linewidth=5, color='black')
    r2Temp = Rectangle((8.5, 0), height=size_z, width=2, fill=True, linestyle='-', linewidth=5, color='black',
                       alpha=0.2)
    r2 = Rectangle((8.5, 0), height=size_z, width=2, fill=False, linestyle='-', linewidth=5, color='black')
    r3Temp = Rectangle((8.5, 0), height=size_z, width=2, fill=True, linestyle='-', linewidth=5, color='black',
                       alpha=0.2)
    r3 = Rectangle((8.5, 0), height=size_z, width=2, fill=False, linestyle='-', linewidth=5, color='black')
    r4Temp = Rectangle((0.5, 0), height=size_z, width=2, fill=True, linestyle='-', linewidth=5, color='black',
                       alpha=0.2)
    r4 = Rectangle((0.5, 0), height=size_z, width=2, fill=False, linestyle='-', linewidth=5, color='black')

    ax.add_patch(r1)
    ax.add_patch(r1Temp)
    ax.add_patch(r2)
    ax.add_patch(r2Temp)
    ax.add_patch(r3)
    ax.add_patch(r3Temp)
    ax.add_patch(r4)
    ax.add_patch(r4Temp)
    art3d.pathpatch_2d_to_3d(r1, z=5.5, zdir="x")
    art3d.pathpatch_2d_to_3d(r1Temp, z=5.5, zdir="x")
    art3d.pathpatch_2d_to_3d(r2, z=5.5, zdir="x")
    art3d.pathpatch_2d_to_3d(r2Temp, z=5.5, zdir="x")
    art3d.pathpatch_2d_to_3d(r3, z=5.5, zdir="y")
    art3d.pathpatch_2d_to_3d(r3Temp, z=5.5, zdir="y")
    art3d.pathpatch_2d_to_3d(r4, z=5.5, zdir="y")
    art3d.pathpatch_2d_to_3d(r4Temp, z=5.5, zdir="y")

    z = np.linspace(0, size_z, 50)
    theta = np.linspace(0, 2 * np.pi, 50)
    thetaGrid, zGrid = np.meshgrid(theta, z)
    xGrid = 3 * np.cos(thetaGrid) + size_x / 2
    yGrid = 3 * np.sin(thetaGrid) + size_y / 2
    ax.plot_surface(xGrid, yGrid, zGrid, alpha=0.2, color='black')


def get_arrow(pt, pt2):
    x, y, z = pt
    u, v, w = pt2 - pt
    return x, y, z, u, v, w


def plotExploredNodes(nodesExplored,ax):
    # if nodesExplored[nList[num]].parent:
    for s in nodesExplored:
        if nodesExplored[s].parent:
            # global quiver
            pt = nodesExplored[s].state
            ptParent = nodesExplored[s].parent.state
            dist = distance(pt, ptParent)
            # quiver.remove()
            # quiver = ax.quiver(*get_arrow(ptParent, pt), length=dist, arrow_length_ratio=0.5, cmap='Reds')
            a = Arrow3D([pt[0], ptParent[0]],
                        [pt[1], ptParent[1]],
                        [pt[2], ptParent[2]], mutation_scale=3,
                        lw=0.8, arrowstyle="-|>", color="blue")
            ax.add_artist(a)


def plotPath(solution,ax):
    # draw solution path
    for i in range(len(solution) - 2, -1, -1):
        pt = solution[i]
        pt1 = solution[i + 1]
        a = Arrow3D([pt[0], pt1[0]],
                    [pt[1], pt1[1]],
                    [pt[2], pt1[2]], mutation_scale=7,
                    lw=1.5, arrowstyle="-|>", color="red")
        ax.add_artist(a)


def drawStartAndGoal():
    u = np.linspace(0, np.pi, 40)
    v = np.linspace(0, 2 * np.pi, 40)

    x = startCoor[0] + 0.2 * np.outer(np.sin(u), np.sin(v))
    y = startCoor[1] + 0.2 * np.outer(np.sin(u), np.cos(v))
    z = startCoor[2] + 0.2 * np.outer(np.cos(u), np.ones_like(v))

    gx = goalCoor[0] + 0.2 * np.outer(np.sin(u), np.sin(v))
    gy = goalCoor[1] + 0.2 * np.outer(np.sin(u), np.cos(v))
    gz = goalCoor[2] + 0.2 * np.outer(np.cos(u), np.ones_like(v))

    ax.plot_surface(x, y, z, color='blue')
    ax.plot_surface(gx, gy, gz, color='orange')


if __name__ == "__main__":
    is1 = -3
    is2 = -3
    is3 = 2
    ig1 = 3
    ig2 = 3
    ig3 = 2
    s1 = 5.5 + is1
    s2 = 5.5 + is2
    s3 = is3
    g1 = 5.5 + ig1
    g2 = 5.5 + ig2
    g3 = ig3

    # ----------------------------
    #  Display parameters
    # ----------------------------
    size_x = 11.0
    size_y = 11.0
    size_z = 5.0

    # ----------------------------
    # Start and goal coordinates
    # ----------------------------
    res = 1
    startCoor = np.float32((np.float32([s1, s2, s3])) / res)
    goalCoor = np.float32((np.float32([g1, g2, g3])) / res)

    startEndCoor = [startCoor, goalCoor]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    q = []
    nodesExplored = {}
    success, solution = generatePath(q, startEndCoor, nodesExplored, 0)
    drawEnv(size_x,size_y,size_z,ax)
    drawStartAndGoal()

    ax.set_xlim3d([0.0, size_x])
    ax.set_xlabel('X')

    ax.set_ylim3d([0.0, size_y])
    ax.set_ylabel('Y')

    ax.set_zlim3d([0.0, size_z])
    ax.set_zlabel('Z')

    ax.set_title('workspace')
    ax.set_facecolor('white')

    numframes = len(nodesExplored)  # +len(solution)
    nList = sorted(nodesExplored.keys())

    # quiver = ax.quiver(*get_arrow(np.array([0, 0]), np.array([0, 0])))
    plotExploredNodes(nodesExplored,ax)
    plotPath(solution,ax)
    plt.show()


