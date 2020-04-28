from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from matplotlib.animation import FuncAnimation
import numpy as np

fig = plt.figure()
ax = fig.gca(projection='3d')

num_frames = 50
theta = np.linspace(0,2*np.pi, 10, endpoint=False)
r = np.arange(1,2.1)
z = np.arange(-2,2.1,1)


def compute_segs(i):
    offset = 2*i*np.pi/num_frames
    theta2,r2, z2 = np.meshgrid(theta+offset,r,z)

    x = r2*np.cos(theta2)
    y = r2*np.sin(theta2)

    u = x+0.2*np.cos(4*theta2)
    v = y
    w = z2+0.2*np.sign(z2)*np.sin(4*theta2)

    return x,y,z2,u,v,w


segs = compute_segs(0)
cols = ['b' for x in segs[0].ravel()]
cols[0] = 'r'
quivers = ax.quiver(*segs, length=0.1, colors = cols, normalize=True)


ax.set_xlim([-3,3])
ax.set_ylim([-3,3])
ax.set_zlim([-3,3])
def animate(i):
    segs = np.array(compute_segs(i)).reshape(6,-1)
    print("---------")
    print(segs)
    print(segs.shape)
    print()
    print()
    print()
    print("====")
    new_segs = [[[x,y,z],[u,v,w]] for x,y,z,u,v,w in zip(*segs.tolist())]
    quivers.set_segments(new_segs)
    return quivers


ani = FuncAnimation(fig, animate, frames = num_frames, interval = 30, blit=False)
ani.save('update_3d_quiver.gif', writer='imagemagick')

plt.show()
