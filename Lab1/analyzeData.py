import h5py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# importing data from data files
data_robot = h5py.File("data_robotTracks.h5", "r")
robot_time = data_robot['time'] # is time the title?
robot_translation = data_robot['t_vecs']
robot_rotation = data_robot['r_vecs']

data_fly = h5py.File("data_flyTracks.h5","r")
fly_time = data_fly['time']
fly_x = data_fly['pos_x']
fly_y = data_fly['pos_y']
fly_angle = data_fly['angle']

# extracting rotation and translation components
robot_x = robot_translation[:,0]
robot_y = robot_translation[:,1]
robot_z = robot_translation[:,2]

robot_rx = robot_rotation[:,0]
robot_ry = robot_rotation[:,1]
robot_rz = robot_rotation[:,2]

# example of plotting data
fig = plt.figure(1)
ax = fig.add_subplot(111, projection = '3d')
ax.scatter(robot_x, robot_y, robot_z)

fig2 = plt.figure(2)
# preliminary fly data analysis
ax2 = fig2.add_subplot(111)
ax2.scatter(fly_x, fly_y)

plt.show()