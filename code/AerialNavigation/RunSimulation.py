from Quadrotor import *
from rrt_3d import *



def followPath(Q,solution):
    drawEnv(size_x,size_y,size_z,ax)
    for i in range(len(solution)):
        ax.cla()
        # drawEnv(size_x,size_y,size_z,ax)
        Q.update_pose(solution[i][0],solution[i][1],solution[i][2],0,0,0)
        plt.pause(0.01)


def runSim():
    nodesExplored = {}
    q = []
    success, solution = generatePath(q, startEndCoor, nodesExplored, 0)
    Q = Quadrotor(ax, x=s1, y=s2, z=s3, roll=roll,
                      pitch=pitch, yaw=yaw, size=1, show_animation=show_animation)


    plotExploredNodes(nodesExplored,ax2)
    plotPath(solution,ax2)
    
    followPath(Q,solution)
    plt.show()

    # for artist in plt.gca().collections:#plt.gca().lines:
         # artist.remove()


#-----------------------------------------------
#.       Quadrotor params
#-----------------------------------------------
s1 = 2.5; s2 = 2.5; s3 = 2;
roll = 0; pitch = 0; yaw = 0 

g1 = 8.5; g2 = 8.5; g3 = 2;
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
startCoor = np.array([s1,s2,s3]) 
goalCoor =  np.array([g1,g2,g3])

startEndCoor = [startCoor, goalCoor]


#----------------------------------------------
#     Environment Setup
#----------------------------------------------
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

drawEnv(size_x,size_y,size_z,ax2)
drawEnv(size_x,size_y,size_z,ax)


#-----------------------------
#   RunSimulation
#-----------------------------
if __name__ =='__main__':
    runSim()





