# -*- coding: utf-8 -*-
"""
Created on Sat Jan 19 13:49:45 2019

@author: pnast
"""
import numpy as np

#Constants
#gravity
g = 9.81;
#mass
m = 0.5;
#
L = 2.5;
# lift constant
k = 3e-6;
#drag constant
b = 1e-7;
#Inertia matrix
I = np.array([[5e-3,0,0],
               [0,5e-3,0], 
               [0,0,10e-3]])
#drag force coefficient
drag = 0.25;

#Initial system state.
#position
x_i = np.array([0, 0, 30])
#velocity
xdot_i = np.array([0, 0, 0])
#euler angles
theta_i = np.array([2, 2, 2])
#angular velocity
thetadot_i = np.array([0, 0, 0])
#initial prop input
props_i = np.array([0,0,0,0])