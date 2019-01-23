# -*- coding: utf-8 -*-
"""
Created on Sat Jan 19 10:27:16 2019

@author: pnast
"""
import numpy as np
import inputsandconstants as innc

#thrust in body frame
def thrust(props,k):
    thr = np.array([0,0, k*(props[0] + props[1] + props[2] + props[3])])
    return thr

# torque in body frame
def torques(props,k,b,L):
    tau = np.array([L*k*(props[3] - props[1]), L*k*(props[2] - props[0]), b * (props[3] - props[2] + props[1] - props[0]) ])
    return tau

# rotate from body frame to lab frame
def rotate_body_to_lab(angles):
    rotation_matrix = np.array([
            [np.cos(angles[2])*np.cos(angles[1]), np.cos(angles[2])*np.sin(angles[1])*np.sin(angles[0]) - np.sin(angles[2])*np.cos(angles[0]), np.cos(angles[2])*np.sin(angles[1])*np.cos(angles[0]) + np.sin(angles[2])*np.sin(angles[0])],
            [np.sin(angles[2])*np.cos(angles[1]), np.sin(angles[2])*np.sin(angles[1])*np.sin(angles[0]) + np.cos(angles[2])*np.cos(angles[0]), np.sin(angles[2])*np.sin(angles[1])*np.cos(angles[0]) - np.cos(angles[2])*np.sin(angles[0])],
            [-1*np.sin(angles[1]), np.cos(angles[1])*np.sin(angles[0]), np.cos(angles[1])*np.cos(angles[0])]])
    return rotation_matrix
    
# Linear acceleration in the lab frame
def acceleration(props,euler_angles,xdot,m,g,k):
    gravity = np.array([0,0,-1*g])
    rotate_body_to_lab(euler_angles)
    thrust(props,k)
    T_lab = rotate_body_to_lab(euler_angles) @ thrust(props,k).T
    F_d = -innc.drag * xdot
    accel = gravity + (1/m)*T_lab + F_d
    return accel
    
#Angular acceleration in body frame
def angular_acceleration(props,omega,inertia_matrix,L,b,k):
    torques(props,k,b,L)
    inertia_inverse = np.linalg.inv(inertia_matrix)
    omegadot = np.dot(inertia_inverse, (torques(props,k,b,L) - np.cross(omega, np.dot(inertia_matrix, omega)))) #check this later
    return omegadot
    
#conversions between theta derivative and angular velocity
def thetadot2omega(thetadot,euler_angles):
    t2o_matrix = np.array([
            [1,0,-1*np.sin(euler_angles[1])],
            [0,np.cos(euler_angles[0]),np.cos(euler_angles[1])*np.sin(euler_angles[0])],
            [0,-1*np.sin(euler_angles[0]),np.cos(euler_angles[1])*np.cos(euler_angles[0])]])
    omeg = np.dot(t2o_matrix,thetadot.T)
    return omeg

def omega2thetadot(omega,euler_angles):
    o2t_matrix = np.array([
            [1,0,-1*np.sin(euler_angles[1])],
            [0,np.cos(euler_angles[0]),np.cos(euler_angles[1])*np.sin(euler_angles[0])],
            [0,-1*np.sin(euler_angles[0]),np.cos(euler_angles[1])*np.cos(euler_angles[0])]])
    thetadot = np.linalg.inv(o2t_matrix) @ omega.T
    return thetadot