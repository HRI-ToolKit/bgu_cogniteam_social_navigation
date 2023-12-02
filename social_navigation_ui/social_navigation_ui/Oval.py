import numpy as np
import matplotlib.pyplot as plt


class Oval:

    def __init__(self, point2d, deg_angle, right_x, left_x, forward_y, backward_y, resolution):
        self.point2d = point2d
        self.deg_angle = deg_angle
        self.right_x = right_x
        self.left_x =   left_x
        self.forward_y = forward_y
        self.backward_y = forward_y
        self.resolution = resolution

        self.points = None

        self.create()
    
    def create(self):

        data = np.array([[0.3, 1.0, -2.0, 3, 2]])

        # Ellipse Vectors, Right x, Left x, Forward y, Backward y
        fig, ax = plt.subplots()
        plt.legend()
        plt.grid(True)        
      
        z = 0.3
        xp = self.right_x
        xn = self.left_x
        yp = self.forward_y
        yn = self.backward_y

        X = []
        Y = []

        x = np.arange(0, xp + self.resolution, self.resolution)
        y = np.sqrt((1 - (x ** 2 / xp ** 2)) * yp ** 2)
        X = np.concatenate([X, x])
        Y = np.concatenate([Y, y])

        x = np.arange(xp, 0 - self.resolution, -self.resolution)
        y = -np.sqrt((1 - (x ** 2 / xp ** 2)) * yn ** 2)
        X = np.concatenate([X, x])
        Y = np.concatenate([Y, y])

        x = np.arange(0, xn - self.resolution, -self.resolution)
        y = -np.sqrt((1 - (x ** 2 / xn ** 2)) * yn ** 2)
        X = np.concatenate([X, x])
        Y = np.concatenate([Y, y])

        x = np.arange(xn, 0 + self.resolution, self.resolution)
        y = np.sqrt((1 - (x ** 2 / xn ** 2)) * yp ** 2)
        X = np.concatenate([X, x])
        Y = np.concatenate([Y, y])

        # Create a list of 2D points from x and y values
        points = list(zip(X, Y))

        # Convert the angle from degrees to radians
        angle_radians = np.radians(self.deg_angle)

        # Perform the rotation
        rotation_matrix = np.array([[np.cos(angle_radians), -np.sin(angle_radians)],
                                    [np.sin(angle_radians), np.cos(angle_radians)]])

        rotated_points = np.dot(points, rotation_matrix.T)

        # Translate each point based on the reference point
        self.points = [(point[0] + self.point2d[0], point[1] + self.point2d[1]) for point in rotated_points]
       
        X_vis, Y_vis = zip(*self.points)

        plt.plot(X_vis, Y_vis, label=f'V in m/s = {z}')
        
        # plt.plot(X, Y, label=f'V in m/s = {z}')


        plt.title('Proxemics Schematic map')
        plt.xlabel('Distance in m')
        plt.ylabel('Distance in m')
        plt.xlim([-10, 10])
        plt.ylim([-10, 10])
        plt.axhline(0, color='black',linewidth=0.5)
        plt.axvline(0, color='black',linewidth=0.5)
        plt.grid(color = 'gray', linestyle = '--', linewidth = 0.5)
        plt.legend()

        plt.show()

oval = Oval((5.0,5.0), 0, 2.0, -2.0, 3, 4, 0.01)