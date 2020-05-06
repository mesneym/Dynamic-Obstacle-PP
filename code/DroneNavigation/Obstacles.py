from Quadrotor import *
import numpy.random as rd

class Obstacles(Quadrotor):
    def __init__(self, ax, x=0, y=0, z=0, roll=0, pitch=0, yaw=0,size= 1,show_animation=True, region = [0,10,0,10]):
                super().__init__(ax, x, y, z, roll, pitch, yaw,size=1,show_animation=True,show_path = False)
                self.region = region


    
    def plot(self):  # pragma: no cover
        T = self.transformation_matrix()

        p1_t = np.matmul(T, self.p1)
        p2_t = np.matmul(T, self.p2)
        p3_t = np.matmul(T, self.p3)
        p4_t = np.matmul(T, self.p4)

        self.ax.plot([p1_t[0], p2_t[0], p3_t[0], p4_t[0]],
                     [p1_t[1], p2_t[1], p3_t[1], p4_t[1]],
                     [p1_t[2], p2_t[2], p3_t[2], p4_t[2]], 'k.')

        self.ax.plot([p1_t[0], p2_t[0]], [p1_t[1], p2_t[1]],
                     [p1_t[2], p2_t[2]], 'g-')
        self.ax.plot([p3_t[0], p4_t[0]], [p3_t[1], p4_t[1]],
                     [p3_t[2], p4_t[2]], 'g-')

        if (self.show_path):
            self.ax.plot(self.x_data, self.y_data, self.z_data, 'b-',linewidth=2)#,linewidth=3,markersize=10)
        self.ax.set_xlim3d([0.0, 11.0])
        self.ax.set_xlabel('X')

        self.ax.set_ylim3d([0.0, 11.0])
        self.ax.set_ylabel('Y')

        self.ax.set_zlim3d([0.0, 5.0])
        self.ax.set_zlabel('Z')


    def move(self):
        x = rd.uniform(self.region[0], self.region[1])
        y = rd.uniform(self.region[2], self.region[3])
        z = rd.uniform(0.0, 5.0)
        self.update_pose(x, y, z, 0, 0, 0)






