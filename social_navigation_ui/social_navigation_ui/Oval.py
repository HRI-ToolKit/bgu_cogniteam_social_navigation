import numpy as np
import matplotlib.pyplot as plt
import math

class Oval:
    def __init__(self, point2d, deg_angle, right_x, left_x, forward_y, backward_y, resolution):
        self.point2d = point2d
        self.deg_angle = deg_angle
        self.right_x = right_x
        self.left_x =   left_x
        self.forward_y = forward_y
        self.backward_y = backward_y
        self.resolution = resolution

        print('point2d ' + str(point2d ))
        print('deg_angle ' + str(deg_angle ))
        print('right_x ' + str(right_x ))
        print('left_x ' + str(left_x ))
        print('forward_y ' + str(forward_y ))
        print('backward_y ' + str(backward_y ))
        print('resolution ' + str(resolution ))


        self.points = None

        self.create()
    
    def create(self):

        # Ellipse Vectors, Right x, Left x, Forward y, Backward y
        fig, ax = plt.subplots()
        plt.legend()
        plt.grid(True)        
      
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
        angle_radians = (np.radians(self.deg_angle))

        # Perform the rotation
        rotation_matrix = np.array([[np.cos(angle_radians), -np.sin(angle_radians)],
                                    [np.sin(angle_radians), np.cos(angle_radians)]])

        rotated_points = np.dot(points, rotation_matrix.T)

        # Translate each point based on the reference point
        self.points = [(point[0] + self.point2d[0], point[1] + self.point2d[1]) for point in rotated_points]
       
        # X_vis, Y_vis = zip(*self.points)

        # plt.plot(X_vis, Y_vis, label=f'V in m/s = {0.3}')
        

        # plt.title('Proxemics Schematic map')
        # plt.xlabel('Distance in m')
        # plt.ylabel('Distance in m')
        # plt.xlim([-10, 10])
        # plt.ylim([-10, 10])
        # plt.axhline(0, color='black',linewidth=0.5)
        # plt.axvline(0, color='black',linewidth=0.5)
        # plt.grid(color = 'gray', linestyle = '--', linewidth = 0.5)
        # plt.legend()

        # plt.show()

    def getOvalPoints(self):
        
        tmp = []
        for p in self.points:
            if math.isnan(p[0]) or math.isnan(p[1]):
                    continue
            tmp.append(p)

        self.points = tmp
        return self.points
        
# oval = Oval((0.0,0.0), 31, 0.25, -0.7, 1, -0.5, 0.04)
# print(oval.getOvalPoints())
# point2d (0.0, 0.0)
# deg_angle 31.930021128213845
# right_x 0.25
# left_x -0.7
# forward_y 1.0
# backward_y -0.5
# resolution 0.04
