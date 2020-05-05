import matplotlib.pyplot as plt
import numpy as np

X, Y = np.meshgrid(np.arange(0, 2 * np.pi, .2), np.arange(0, 2 * np.pi, .2))
U = np.cos(X)
V = np.sin(Y)

plt.figure()
plt.title('Arrows scale with plot width, not view')
plt.xlabel('xlabel')
plt.xlabel('ylabel')

Q = plt.quiver(X, Y, U, V, units='width')
l, = plt.plot(X[0,:], U[4,:]+2)


# option 1, remove single artists
#Q.remove()
#l.remove()

# option 2, remove all lines and collections
# for artist in plt.gca().lines + plt.gca().collections:
    # artist.remove()

for artist in plt.gca().collections:#plt.gca().lines:
    artist.remove()

# plt.cla()
plt.show()
