import os
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def readGraph(filename):
    '''read g2o file'''
    data_array = np.matrix([[0, 0, 0]])
    with open(filename, 'r') as fh:
        for line in fh:
            try:
                data_type, nodeID, x, y, z, qx, qy, qz, qw = line.split()
                if data_type == "VERTEX_SE3:QUAT":
                    data_array = np.append(data_array, [[float(x), float(y), float(z)]], axis=0)
            except:
                break
    
    return data_array


robot_traj = readGraph('/home/ziwei/Downloads/test.g2o')
z = np.array(robot_traj[:,2])
# print(robot_traj[:,0].shape, z.shape)
# z = np.broadcast_to(z, len(robot_traj[:,0]))
fig = plt.figure()
fig.suptitle('cubicle')
ax1 = Axes3D(fig)
ax1.scatter(robot_traj[:,0], robot_traj[:,1], zs=z, label="Estimate")
plt.ylabel('Y (m)')
plt.xlabel('X (m)')
plt.show()