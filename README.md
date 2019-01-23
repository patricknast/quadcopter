# quadcopter
This is a simple mathematical model of the dynamics and control of an idealized quadcopter implemented in Python. 
I wrote this up over 1/19/19-1/21/19 as a way to learn more about the physics of Quadrotor systems, PID control, and general python programming.
This project is neither complete, nor very clean as a result of time constraints and novice understanding.

Essentially what the model does is attempt to reduce the euler angles to zero given some starting attitude, gravity, and air resistance proportional to velocity.
The thrust is intended to stabilize the quadcopter at a consistent altitude. Control of the x and y positions is not implemented, so the quadrotor tends to drift a lot.
Most models take IMUs as the only sensor but I simplified the controller to just assume that the quadrotor has a precise understanding
of its attitude and position through some advanced, unspecified sensing stack.

There are tons of little things I would like to fix / expand on in the future but this was as far as I got in my spare time.
Specifically, I'd probably normalize the euler angles to [-pi,pi] and add some large-scale randomly generated input attitude simulations
to assess the perfomance of the controller and tune the constants for the PID controls.

I've added a screenshot of a few simulations running---a couple worked OK but you can see that the controller will oscillate wildly and return
NaNs if the displacements are too large.

This project was lovingly (but shamelessly) cribbed largely from a few sources, especially the quadrotor paper and code repository 
created by Andrew Gibiansky, who used Matlab:

http://andrew.gibiansky.com/downloads/pdf/Quadcopter%20Dynamics,%20Simulation,%20and%20Control.pdf
https://github.com/gibiansky/experiments/tree/master/quadcopter

Others:
http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf

https://github.com/noether/pycopter 
--For references on python implementation

--Countless visits to StackExchange and python documentation.
