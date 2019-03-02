#!/usr/bin/env python2

from SawyerClass import Sawyer
import sympy as sp
import numpy as np
import sys
import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d import Axes3D

q = np.loadtxt(sys.argv[1], skiprows=1, delimiter=',') # todo: might not default to command for del

robot = Sawyer()

Te = sp.lambdify(robot.q, robot.get_T_f()[-1])

cart = np.array([Te(*g[1:8])[:-1,3] for g in q])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(cart[:,0], cart[:,1], cart[:,2], 'b')
ax.text2D(0.05, 0.95, sys.argv[1].split('/')[1].split('.')[0].replace('-',' '), transform=ax.transAxes)
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
plt.show()
