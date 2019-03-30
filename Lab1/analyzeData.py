import h5py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

f = h5py.File("data0_1.h5","r")
print(f)
time = f['time']
x_pos = f['pos_x']
y_pos = f['pos_y']
angle = f['angle']

fig = plt.figure(1)
plt.scatter(x_pos, y_pos)
plt.grid('on')
plt.show()