# -*- coding: utf-8 -*-
"""
Created on Sat Jan 19 13:57:53 2019

@author: not pnast
"""
import dynamics as dyn
import simulator as sim
import inputsandconstants as innc
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
    
# Simulation parameters
tf = 10
dt = 1e-2
time = np.linspace(0.0, tf, tf/dt)
it = 0
frames = 1


#Initialize a quadcopter to the simulation
quad1 = sim.quadcopter(innc.x_i,innc.xdot_i,innc.theta_i,innc.thetadot_i,innc.props_i)


#quadcopter model
def model(position,angles,L):
    body_arrows = np.array([[L,0,0],
                            [-L,0,0],
                            [0,L,0],
                            [0,-L,0]])
    lab_arrows = np.dot(dyn.rotate_body_to_lab(angles), body_arrows.T)
    ax.quiver(position[0], position[1], position[2], lab_arrows[0][0], lab_arrows[1][0],lab_arrows[2][0], length=L, normalize=True,color='red')
    ax.quiver(position[0], position[1], position[2], lab_arrows[0][1], lab_arrows[1][1],lab_arrows[2][1], length=L, normalize=True,color='green')
    ax.quiver(position[0], position[1], position[2], lab_arrows[0][2], lab_arrows[1][2],lab_arrows[2][2], length=L, normalize=True,color='blue')
    ax.quiver(position[0], position[1], position[2], lab_arrows[0][3], lab_arrows[1][3],lab_arrows[2][3], length=L, normalize=True,color='yellow')
    
for t in time:

    # Simulation
    quad1.step(dt)
    quad1.control_step(dt)
    x = quad1.x
    theta = quad1.theta
    #print('position')
    #print(x)
    print('angle')
    print(theta)
    print('inputs')
    print(quad1.prop_in)
    # Animation
    if it%frames == 0:

        ax.cla()
        model(x,theta,innc.L)
        ax.set_xlim3d([-100.0, 100.0])
        ax.set_xlabel('X')

        ax.set_ylim3d([-100.0, 100.0])
        ax.set_ylabel('Y')

        ax.set_zlim3d([0.0, 100.0])
        ax.set_zlabel('Z')
        plt.pause(0.01)
        plt.draw()
    it+=1


