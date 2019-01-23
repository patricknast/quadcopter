# -*- coding: utf-8 -*-
"""
Created on Sat Jan 19 13:50:29 2019

@author: pnast
"""
import inputsandconstants as innc
import numpy as np
import dynamics as dyn


#Physics and Control 



class quadcopter:
    def __init__(self,xi,xdoti,thetai,thetadoti,propsi):
        #Position, velocity, angular position, angular velocity
        self.x = xi
        self.xdot = xdoti
        self.theta = thetai
        self.thetadot = thetadoti
        
        #prop inputs (omega squared)
        self.c1 = propsi[0]
        self.c2 = propsi[1]
        self.c3 = propsi[2]
        self.c4 = propsi[3]
        self.prop_in = propsi
        
        #PID parameters
        # TODO: account for integral wind-up
        self.kd = 3
        self.kp = 2
        self.ki = 0 # Just PD for now
        
        #Initial Desired angles / angular velocities and initial error
        self.ang_d = np.array([0, 0, 0])
        self.angv_d = np.array([0, 0, 0])
        self.errors = np.array([0, 0, 0])

    def step(self,dt):
        # take a step in the simulation using Euler method.
     
        # forces, torques, accels
        omega = dyn.thetadot2omega(self.thetadot, self.theta)
        a = dyn.acceleration(self.prop_in,self.theta,innc.drag,innc.m,innc.g,innc.k)
        omegadot =dyn.angular_acceleration(self.prop_in,omega,innc.I,innc.L,innc.b,innc.k)
    
        # advance system state
        omega = omega + dt * omegadot
        self.thetadot = dyn.omega2thetadot(omega, self.theta)
        self.theta = self.theta + dt * self.thetadot
        self.xdot = self.xdot + dt * a
        self.x = self.x + dt * self.xdot
    
        #return self.x
        #return self.xdot
        #return self.theta
        #return self.thetadot

    def control_step(self,dt):
        # compute errors, calculate inputs, advance controller
        self.errors = self.kd * (self.thetadot - self.angv_d) + self.kp * (self.theta-self.ang_d) + self.ki * (self.theta - self.ang_d)* dt
        quadcopter.inputs(self)
   
    def inputs(self):
        #compute the desired input given values of angular error.
        
        
       # error_x = posx - pos_xd
        #error_y = posy - pos_yd
       # error_z = posz - pos_zd
        error_phi = self.errors[0]
        error_theta = self.errors[1]
        error_psi = self.errors[2]
        
        #Thrust
        thrust = innc.m * innc.g / (innc.k * np.cos(self.theta[0])* np.cos(self.theta[1]))
        
        #Props
        self.c1 = thrust/4 + (error_theta*innc.I[1][1])/(2*innc.k*innc.L) + (error_psi*innc.I[2][2])/(4*innc.b)
        self.c2 = thrust/4 + (error_phi*innc.I[0][0])/(2*innc.k*innc.L) - (error_psi*innc.I[2][2])/(4*innc.b)
        self.c3 = thrust/4 - (error_theta*innc.I[1][1])/(2*innc.k*innc.L) + (error_psi*innc.I[2][2])/(4*innc.b)
        self.c4 = thrust/4 - (error_phi*innc.I[0][0])/(2*innc.k*innc.L) - (error_psi*innc.I[2][2])/(4*innc.b)
        self.prop_in = np.array([self.c1,self.c2,self.c3,self.c4])