import pygame
import numpy as np
from Astar import *
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, PathPatch,Rectangle
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.art3d as art3d


def writeSolutionToFile(solution):
    # Writing all explored nodes in text file.
    sol = np.array(solution)

    #converting from image coordinates to world cooridnates
    sol[:,0] = -5 + sol[:,0]
    sol[:,1] = 5 - sol[:,1]
    sol[:,2] = 360 - sol[:,2]
    
    #Solution is stored as goal to start so we reverse it
    for i in range(6):
        sol[:,i] = -1*sol[:,i][::-1]
    np.savetxt('solution.txt',sol, delimiter=',')


def writeParametersForGazebo(dt,is1,is2,iorientation):
    #input from world coordinates to gazebo coordinates 
    # s1 = -is1
    # s2 = -is2
    s1 = is1
    s2 = is2
    orientation = math.radians(iorientation)# + math.pi #gazebo works in radians

    params = np.array([dt,s1,s2,orientation]) 
    np.savetxt('gazebo_params.txt',params,delimiter=',')


def drawEnv():
    c2 = Circle((size_x/2,size_y/2), 3,fill=True,linestyle='-',linewidth=5,color='black',alpha=0.2)
    c3 = Circle((size_x/2,size_y/2), 5,fill=True,linestyle='-',linewidth=5,fc='black',ec='black',alpha=0.2)
    c1 = Circle((size_x/2,size_y/2), 4,fill=False,linestyle='--',linewidth=5,color='white')
    c2Temp = Circle((size_x/2,size_y/2), 3,fill=False,linestyle='-',linewidth=5,color='black')
    c3Temp = Circle((size_x/2,size_y/2), 5,fill=False,linestyle='-',linewidth=5,color='black')
    
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


    r1Temp = Rectangle((0.5,0),height=size_z,width=2,fill=True,linestyle='-',linewidth=5,color='black',alpha=0.2) 
    r1 = Rectangle((0.5,0),height=size_z,width=2,fill=False,linestyle='-',linewidth=5,color='white') 
    r2Temp = Rectangle((8.5,0),height=size_z,width=2,fill=True,linestyle='-',linewidth=5,color='black',alpha=0.2) 
    r2 = Rectangle((8.5,0),height=size_z,width=2,fill=False,linestyle='-',linewidth=5,color='white') 
    r3Temp = Rectangle((8.5,0),height=size_z,width=2,fill=True,linestyle='-',linewidth=5,color='black',alpha=0.2) 
    r3 = Rectangle((8.5,0),height=size_z,width=2,fill=False,linestyle='-',linewidth=5,color='white') 
    r4Temp = Rectangle((0.5,0),height=size_z,width=2,fill=True,linestyle='-',linewidth=5,color='black',alpha=0.2) 
    r4 = Rectangle((0.5,0),height=size_z,width=2,fill=False,linestyle='-',linewidth=5,color='white') 

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


    z = np.linspace(0,size_z,50)
    theta = np.linspace(0,2*np.pi,50)
    thetaGrid,zGrid = np.meshgrid(theta,z)
    xGrid =3*np.cos(thetaGrid)+size_x/2
    yGrid =3*np.sin(thetaGrid)+size_y/2
    ax.plot_surface(xGrid,yGrid,zGrid,alpha=0.2,color='black')



###################################################
#                  Parameters 
###################################################
#------------------------------
#  Getting user Inputs
#------------------------------
# clearance = 0
# print('Robot considered is Turtlebot 2:')
# print("Enter cleareance")
# clearance = float(input())

# print('Enter start location s1 between -5 and 5')
# is1 = float(input())
# print('Enter start location s2 between -5 and 5')
# is2 = float(input())
# print('Enter the angle of the robot in degrees')
# istartOrientation = float(input())

# print('Enter goal location g1 between -5 and 5')
# ig1 = float(input())
# print('Enter goal location g2 between -5 and 5')
# ig2 = float(input())

# print('Enter left wheel rotational velocity')
# iul = float(input())
# print('Enter right wheel rotational velocity')
# iur = float(input())

# print('Enter smooth Coef or negative value for default paramater')
# ismoothCoef = float(input())

# print('Enter Time step or negative value for default paramater')
# idt = float(input())

iul = 20
iur = 20
is1 = -4#-4  #-4   
is2 = -4#-4  #-3     
ig1 = 4#4   #0       
ig2 = 2.5#2.5  #-3    
istartOrientation = 0
idt = 0.6 #0.8
ismoothCoef =  0.2 #0.1

#---------------------------------
# Inputs From World Coordinates 
# To Pygame Coordinates
#---------------------------------
startOrientation = 360 - istartOrientation
ul = iul
ur = iur
s1 = 5+(is1)
s2 = 5-(is2)
g1 = 5+(ig1)
g2 = 5-(ig2)
dt = idt if(idt>=0.0) else  0.3
smoothCoef = ismoothCoef if (ismoothCoef>= 0) else 0.5

#---------------------------
#  Precision Parameters
#---------------------------
threshDistance = 0.1
clearance = 0.3
threshAngle = 5

#---------------------------
#  Robot parameters
#---------------------------
# smoothCoef = ismoothCoef if (ismoothCoef>= 0) else 0.5
wheelDist = 0.2116 # 0.3175/6 * 4
wheelRadius = 0.038
robotParams = [ul,ur,wheelRadius,wheelDist, smoothCoef]
robotRadius = 0.177

#-------------------------------
#  Parameters needed by gazebo
#-------------------------------
#dt - affects publishing rate
#   - dt must be of resolution 0.1 
#   - restricting frequency to 10Hz in gazebo
#   - 1/frequency*dt must be a whole number 

#is1,is2,iorientation- initial pose for robot
writeParametersForGazebo(dt,is1,is2,istartOrientation)

#----------------------------
#  Display parameters
#----------------------------
size_x = 11.0
size_y = 11.0
size_z = 5.0

#----------------------------
# Start and goal coordinates
#----------------------------
res = 1
startCoor = np.float32((np.float32([s1,s2,startOrientation]))/res)
goalCoor = np.float32((np.float32([g1,g2,0]))/res)

startEndCoor = [startCoor,goalCoor]

    
def drawStartAndGoal():
    u = np.linspace(0, np.pi, 40)
    v = np.linspace(0, 2 * np.pi, 40)
    
    x = startCoor[0] + 0.2*np.outer(np.sin(u), np.sin(v))
    y = startCoor[1] + 0.2*np.outer(np.sin(u), np.cos(v))
    z = startCoor[2] + 0.2*np.outer(np.cos(u), np.ones_like(v)) 
    
    gx = goalCoor[0] + 0.2*np.outer(np.sin(u), np.sin(v))
    gy = goalCoor[1] + 0.2*np.outer(np.sin(u), np.cos(v))
    gz = goalCoor[2] + 0.2*np.outer(np.cos(u), np.ones_like(v)) 

    ax.plot_surface(x,y,z,color='blue')
    ax.plot_surface(gx,gy,gz,color='orange')


def drawObstacles(num):
    pass

def drawExploredNodes():

    pass

def drawOptimalPath():
    pass


def get_arrow(pt,pt2):
    x,y = pt
    u,v = pt2-pt
    return x,y,2,u,v,0

def update(num,nodesExplored,nList):
    if(nodesExplored[nList[num]].parent):
        global quiver
        pt = nodesExplored[nList[num]].state[0:2]
        ptParent = nodesExplored[nList[num]].parent.state[0:2]
        # quiver.remove()
        quiver = ax.quiver(*get_arrow(ptParent,pt))


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


q = []
nodesExplored = {}
success,solution = generatePath(q,startEndCoor,nodesExplored,robotParams,dt,clearance+robotRadius,threshDistance,threshAngle)
drawEnv()
drawStartAndGoal()

ax.set_xlim3d([0.0, size_x])
ax.set_xlabel('X')

ax.set_ylim3d([0.0, size_y])
ax.set_ylabel('Y')

ax.set_zlim3d([0.0, size_z])
ax.set_zlabel('Z')

ax.set_title('workspace')
ax.set_facecolor('orange')


numframes= len(nodesExplored)#+len(solution)
nList = sorted(nodesExplored.keys())

print(numframes)
quiver = ax.quiver(*get_arrow(np.array([0,0]),np.array([0,0])))
ani = animation.FuncAnimation(fig, update,numframes, fargs=(nodesExplored,nList),interval=1, blit=False)
plt.show()




















# Fixing random state for reproducibility
# np.random.seed(19680801)


# def Gen_RandLine(length, dims=2):
    # """
    # Create a line using a random walk algorithm

    # length is the number of points for the line.
    # dims is the number of dimensions the line has.
    # """
    # lineData = np.empty((dims, length))
    # lineData[:, 0] = np.random.rand(dims)
    # for index in range(1, length):
        # scaling the random numbers by 0.1 so
        # movement is small compared to position.
        # subtraction by 0.5 is to change the range to [-0.5, 0.5]
        # to allow a line to move backwards.
        # step = ((np.random.rand(dims) - 0.5) * 1)
        # lineData[:, index] = lineData[:, index - 1] + step

    # return lineData


# def update_lines(num, dataLines, lines):
    # print(num)
    # print("======")
    # for line, data in zip(lines, dataLines):
        # NOTE: there is no .set_data() for 3 dim data...
        # print(data[0:2,:num])
        # line.set_data(data[0:2, :num])
        # line.set_3d_properties(data[2, :num])
    # return lines

# Attaching 3D axis to the figure
# fig = plt.figure()
# ax = p3.Axes3D(fig)

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# Make data
# u = np.linspace(0, 2 * np.pi, 100)
# v = np.linspace(0, np.pi, 100)
# x = 10 * np.outer(np.cos(u), np.sin(v))
# y = 10 * np.outer(np.sin(u), np.sin(v))
# z = 10 * np.outer(np.ones(np.size(u)), np.cos(v))

# Plot the surface
# ax.plot_surface(x, y, z, color='b')


# Fifty lines of random 3-D lines
# data = [Gen_RandLine(25, 3) for index in range(2)]

# Creating fifty line objects.
# NOTE: Can't pass empty arrays into 3d version of plot()
# lines = [ax.plot(dat[0, 0:1], dat[1, 0:1], dat[2, 0:1])[0] for dat in data]

# x = [(dat[0,0],dat[1,0],dat[2,0]) for dat in data]

# Setting the axes properties
# ax.set_xlim3d([0.0, 11.0])
# ax.set_xlabel('X')

# ax.set_ylim3d([0.0, 11.0])
# ax.set_ylabel('Y')

# ax.set_zlim3d([0.0, 4.0])
# ax.set_zlabel('Z')

# ax.set_title('3D Test')


# Creating the Animation object
# a = np.arange(25)
# line_ani = animation.FuncAnimation(fig, update_lines,a, fargs=(data, lines),
                                   # interval=50, blit=True)
# plt.show()




############################################################
#                 Display Obstacles
############################################################
# circlePts1 = [6,6,4]  # middle circle
# circlePts2 = [6,6,5]  # outer circle
# circlePts3 = [6,6,3]  # inner circle


# pygame.draw.circle(gameDisplay, white, (circlePts1[0]*scale,circlePts1[1]*scale), circlePts1[2]*scale,2)
# pygame.draw.circle(gameDisplay, white, (circlePts2[0]*scale,circlePts2[1]*scale), circlePts2[2]*scale,2)
# pygame.draw.circle(gameDisplay, red, (circlePts3[0]*scale,circlePts3[1]*scale), circlePts3[2]*scale)

# pygame.draw.rect(gameDisplay,white,[scale*9,scale*5,scale*1.9,scale*0.5])
# pygame.draw.rect(gameDisplay,white,[scale*9,scale*6,scale*1.9,scale*0.5])
# pygame.draw.rect(gameDisplay,white,[scale*(9+0.63),scale*5.5,scale*0.5,scale*0.5])
# pygame.draw.rect(gameDisplay,white,[scale*(9),scale*5.5,scale*0.2,scale*0.5])
# pygame.draw.rect(gameDisplay,white,[scale*(9+1.5),scale*5.5,scale*0.2,scale*0.5])
# pygame.draw.rect(gameDisplay,white,[scale*9,scale*6,scale*1.9,scale*0.3])

# pygame.draw.rect(gameDisplay,green,[scale*1.1,scale*5,scale*1.9,scale*1.9])
# pygame.draw.rect(gameDisplay,green,[scale*5,scale*1.1,scale*1.9,scale*1.9])
# pygame.draw.rect(gameDisplay,green,[scale*5,scale*9,scale*1.9,scale*1.9])



###########################################################
         # Draw Explored Nodes and solution path
###########################################################
# nodesExplored = {}
# q = []

# if(not isSafe(startCoor,res,clearance + robotRadius) or not isSafe(goalCoor,res,clearance + robotRadius)):
    # pygame.draw.rect(gameDisplay,blue,(startCoor[0]*res*scale,startCoor[1]*res*scale, \
                                 # res*2,res*2))

    # pygame.draw.circle(gameDisplay,blue,(int(goalCoor[0]*res*scale),int(goalCoor[1]*res*scale)), \
                                  # math.floor(0.3*res*scale))

    # pygame.draw.rect(gameDisplay,white,(goalCoor[0]*res*scale,goalCoor[1]*res*scale, \
                                 # res*2,res*2))
    # basicfont = pygame.font.SysFont(None, 48)
    # text = basicfont.render('Start or goal position must be in a valid workspace', True, (255, 0, 0), (255, 255, 255))
    # textrect = text.get_rect()
    # textrect.centerx = gameDisplay.get_rect().centerx
    # textrect.centery = gameDisplay.get_rect().centery
 
    # gameDisplay.blit(text, textrect)
    # pygame.display.update()
    # pygame.time.delay(2000)

# else:
    # startTime = time.time()  # Start time of simulation
    # print('Exploring nodes...')
    # success,solution = generatePath(q,startEndCoor,nodesExplored,robotParams,dt,clearance+robotRadius,threshDistance,threshAngle)
    # success = False
    # endTime= time.time()
    
    ############################################
         # Drawing 
    ############################################
    # if(success):
        # print('Optimal path found')
        # print("Total time taken for exploring nodes "+ str(endTime-startTime) +" seconds.")
        # writeSolutionToFile(solution)

        # draw = True
        # while draw:
            # for event in pygame.event.get():
                # if event.type == pygame.QUIT:
                    # pygame.quit()
                    # quit()
             
            # draw nodesExplored
            # for s in nodesExplored:
                # if(nodesExplored[s].parent):
                    # pt = nodesExplored[s].state[0:2]
                    # ptParent = nodesExplored[s].parent.state[0:2]
                    # x,y = pt*scale*res
                    # x2,y2 = ptParent*scale*res

                    # draw explored nodes
                    # pygame.draw.line(gameDisplay,white,(x2,y2),(x,y),1)
                    # pygame.draw.circle(gameDisplay,green,(int(x),int(y)),4)
                    # triangle = triangleCoordinates([x2,y2],[x,y],5)
                    # pygame.draw.polygon(gameDisplay, green,[tuple(triangle[0]),tuple(triangle[1]),tuple(triangle[2])])

                # draw start and goal locations
                # pygame.draw.rect(gameDisplay,blue,(startCoor[0]*res*scale,startCoor[1]*res*scale, \
                                 # res*2,res*2))

                # pygame.draw.circle(gameDisplay,blue,(int(goalCoor[0]*res*scale),int(goalCoor[1]*res*scale)), \
                                  # math.floor(0.3*res*scale))

                # pygame.draw.rect(gameDisplay,white,(goalCoor[0]*res*scale,goalCoor[1]*res*scale, \
                                 # res*2,res*2))
                # pygame.display.update()
           
            # draw solution path
            # for i in range(len(solution)-2,-1,-1):
                # pt = solution[i][0:2]
                # pt1 = solution[i+1][0:2]
                # xt,yt = pt[0]*scale*res,pt[1]*scale*res
                # x, y = pt1[0]*scale*res,pt1[1]*scale*res
                # pygame.draw.line(gameDisplay,yellow,(xt,yt),(x,y),3)
                # pygame.draw.circle(gameDisplay,red,(int(x),int(y)),4)
                # pygame.display.update()
            # pygame.time.delay(4000)
            # draw = False

    # else:
        # print('Path not possible')
        # basicfont = pygame.font.SysFont(None, 48)
        # text = basicfont.render('Path can\'t be generated', True, (255, 0, 0), (255, 255, 255))
        # textrect = text.get_rect()
        # textrect.centerx = gameDisplay.get_rect().centerx
        # textrect.centery = gameDisplay.get_rect().centery
     
        # gameDisplay.blit(text, textrect)
        # pygame.display.update()
        # pygame.time.delay(4000)

# pygame.quit()

