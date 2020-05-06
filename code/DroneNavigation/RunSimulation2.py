from Quadrotor import *
from Obstacles import *
from rrt_3d import *


def moveObstacles(obstacles):
    for i in range(len(obstacles)):
        obstacles[i].move()


def collisionCheck(Obs, Obsindex, Q):
    Obsvec = np.array([Obs[Obsindex].x_data[-1], Obs[Obsindex].y_data[-1], Obs[Obsindex].z_data[-1]]) - \
             np.array([Obs[Obsindex].x_data[-2], Obs[Obsindex].y_data[-2], Obs[Obsindex].z_data[-2]])

    Qvec = np.array([Q.x_data[-1], Q.y_data[-1], Q.z_data[-1]]) - np.array([Q.x_data[-2], Q.y_data[-2], Q.z_data[-2]])
    angle = np.dot(Obsvec, Qvec.T) / (np.linalg.norm(Obsvec) * np.linalg.norm(Qvec))
    if -0.88387747318 < angle < 0.69925080647:
        return True
    return False


def pathObstruction(O, Q, solution, index):
    Qpos = np.array([Q.x_data[-1], Q.y_data[-1], Q.z_data[-1]])

    for i in range(index, len(solution)):
        for j in range(len(O)):
            Opos = np.array([O[j].x, O[j].y, O[j].z])
            solpos = np.array(solution[i])
            solDist = np.sqrt(np.sum((solpos - Opos) ** 2))
            obstacleIsNear = np.sqrt(np.sum((Qpos - Opos) ** 2)) < 1
            if obstacleIsNear and solDist < 1:
                # if collisionCheck(O, j, Q):
                return True
    return False


def replan(q, startCoor, GoalCoor, nodesExplored, radiusClearance):
    startEndCoor = [startCoor, GoalCoor]
    success, solution = generatePath(q, startEndCoor, {}, radiusClearance)
    solution.reverse()
    resetEnv()
    plt.pause(0.01)
    plotExploredNodes(nodesExplored, ax2)
    plotPath(solution, ax2)
    plt.pause(0.01)
    return success, solution


def resetEnv():
    ax2.cla()
    ax2.set_xlim3d([0.0, size_x])
    ax2.set_xlabel('X')

    ax2.set_ylim3d([0.0, size_y])
    ax2.set_ylabel('Y')

    ax2.set_zlim3d([0.0, size_z])
    ax2.set_zlabel('Z')

    ax2.set_title('Replanning')
    ax2.set_facecolor('white')
    drawEnv(size_x, size_y, size_z, ax2)


def followPath(Q, solution, O, q, nodesExplored, radiusClearance):
    drawEnv(size_x, size_y, size_z, ax)
    N = len(solution);
    i = 0;
    count = 0
    while i < N:
        ax.cla()
        drawEnv(size_x, size_y, size_z, ax)
        moveObstacles(O)
        Q.update_pose(solution[i][0], solution[i][1], solution[i][2], 0, 0, 0)

        if pathObstruction(O, Q, solution, i):
            print(len(solution))
            success, solution = replan(q, solution[i], solution[-1], nodesExplored, radiusClearance)
            i = 0
            N = len(solution)

        plt.pause(0.01)
        i += 1
        count += 1


def runSim():
    nodesExplored = {}
    q = []
    success, solution = generatePath(q, startEndCoor, nodesExplored, 0)
    solution.reverse()
    Q = Quadrotor(ax, x=s1, y=s2, z=s3, roll=roll,
                  pitch=pitch, yaw=yaw, size=1, show_animation=show_animation)

    O = {}
    region = [[0, 5, 0, 5], [6, 10, 0, 5], [0, 5, 6, 10], [6, 10, 6, 10]]
    for i in range(4):
        O[i] = Obstacles(ax, x=region[i][0], y=region[i][3], z=2.5, roll=roll,
                         pitch=pitch, yaw=yaw, size=1, show_animation=show_animation, region=region[i])

    plotExploredNodes(nodesExplored, ax2)
    plotPath(solution, ax2)
    followPath(Q, solution, O, q, nodesExplored, 0)

    # for artist in plt.gca().collections:#plt.gca().lines:
    # artist.remove()


# -----------------------------------------------
# .       Quadrotor params
# -----------------------------------------------
s1 = 2.5;
s2 = 2.5;
s3 = 2;
roll = 0;
pitch = 0;
yaw = 0

g1 = 8.5;
g2 = 8.5;
g3 = 2;
show_animation = True

# ----------------------------
#  Display parameters
# ----------------------------
size_x = 11.0
size_y = 11.0
size_z = 5.0

# ----------------------------
# Start and goal coordinates
# ----------------------------
startCoor = np.array([s1, s2, s3])
goalCoor = np.array([g1, g2, g3])

startEndCoor = [startCoor, goalCoor]

# ----------------------------------------------
#     Environment Setup
# ----------------------------------------------
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

fig2 = plt.figure()
ax2 = fig2.add_subplot(111, projection='3d')

ax.set_xlim3d([0.0, size_x])
ax.set_xlabel('X')

ax.set_ylim3d([0.0, size_y])
ax.set_ylabel('Y')

ax.set_zlim3d([0.0, size_z])
ax.set_zlabel('Z')

ax.set_title('Drone Path')
ax.set_facecolor('white')

ax2.set_xlim3d([0.0, size_x])
ax2.set_xlabel('X')

ax2.set_ylim3d([0.0, size_y])
ax2.set_ylabel('Y')

ax2.set_zlim3d([0.0, size_z])
ax2.set_zlabel('Z')

ax2.set_title('ExploredNodes')
ax2.set_facecolor('white')

drawEnv(size_x, size_y, size_z, ax2)
drawEnv(size_x, size_y, size_z, ax)

# -----------------------------
#   RunSimulation
# -----------------------------
if __name__ == '__main__':
    runSim()
