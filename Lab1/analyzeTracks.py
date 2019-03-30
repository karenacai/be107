import h5py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

f = h5py.File("data.h5","r")

dset = f['t_vecs']

x = []
y = []
z = []
for a in dset:
    x+=[a[0]]
    y+=[a[1]]
    z+=[a[2]]
    
fig = plt.figure()
ax = Axes3D(fig)
ax.plot(x,y,z)

plt.show()
