import numpy as np
import matplotlib.pyplot as plt

data = np.array([
    [0.3, 0.25, -0.7, 1, -0.5]
])

# Tested Vector, Right x, Left x, Forward y, Backward y
plt.figure()
res = 0.05

for i in range(data.shape[0]):
    print(f" i {i}")
    z = data[i, 0]
    xp = 0.5 #data[i, 1]
    xn = -0.8 # data[i, 2]
    yp = 3.6 # data[i, 3]
    yn = -0.8 # data[i, 4]

    #   xp = self.right_x
    #     xn = self.left_x
    #     yp = self.forward_y
    #     yn = self.backward_y

    X = []
    Y = []

    x = np.arange(0, xp + res, res)
    y = np.sqrt(np.maximum(0, (1 - (x**2 / xp**2)) * yp**2))
    X = np.concatenate([X, x])
    Y = np.concatenate([Y, y])

    x = np.arange(xp, 0 - res, -res)
    y = -np.sqrt(np.maximum(0, (1 - (x**2 / xp**2)) * yn**2))
    X = np.concatenate([X, x])
    Y = np.concatenate([Y, y])

    x = np.arange(0, xn - res, -res)
    y = -np.sqrt(np.maximum(0, (1 - (x**2 / xn**2)) * yn**2))
    X = np.concatenate([X, x])
    Y = np.concatenate([Y, y])

    x = np.arange(xn, 0 + res, res)
    y = np.sqrt(np.maximum(0, (1 - (x**2 / xn**2)) * yp**2))
    X = np.concatenate([X, x])
    Y = np.concatenate([Y, y])

    plt.plot(X, Y, label=f'V in m/s = {z}')

    plt.title('Proxemics Schematic map')
    plt.xlabel('Distance in m')
    plt.ylabel('Distance in m')
    plt.xlim([-10.5, 10.5])
    plt.ylim([-10.5, 10.5])

    ax = plt.gca()
    ax.spines['left'].set_position('zero')
    ax.spines['bottom'].set_position('zero')
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')
    ax.xaxis.labelpad = 150
    ax.yaxis.labelpad = 150

plt.legend(title='Legend', bbox_to_anchor=(1.1, 1.1), loc='upper right')

plt.show()