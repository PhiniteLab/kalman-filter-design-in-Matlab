# kalman-filter-design-in-Matlab

KALMAN FILTER DESIGN IN MATLAB SCRIPT/SIMULINK

 Solver design especially in the computer simulation is an important matter to emulate any physical dynamical system. In order to understand the underlying mechanism of this thing, there should be a model needs to be developed in terms of differential equation and matrices equation.
 
 In this study, system dynamic library is developed in order to simulate any kinds of systems with respect to the given time parameters and system models.
 
 Also, the main course of this github repository includes the bare metal kalman filter design in Matlab.
 
 There are four folders in this repository:
 
 - Learning : standard matlab kalman filter based on forward difference solver in linear domain (MCK system)
 - Nonlinear : nonlinear system example with van der pol equation, jacobian usage and real time update
 - simulink : linear and nonlinear system applications given in the folder of nonliner/learning, MCK and Van der pol equation are performed!
 - unmodeledKalman : this is the simplest real time low pass filter approach on the data coming from the unknown dynamic
 
 There are five main usage of this library:
 
 - filtering the any signal on embedded systems
 - system identification and estimation
 - simulation of the modeled system
 - obtaining time response characteristics of the system
 - creating solver design to be implemented on embedded systems

  
Which kinds of projects you can utilize the basis of this codes?

 - Engine design (physic motors)
 - VR/AR application
 - Embedded software simulation
 - Estimation and Control of Physical Systems
 - Real time engine application

How can you use the library?

 - Suppose that you have a system like
  xdot = A*x + B*u
   with the parameter of A matrice (state matrice) and B matrice (input matrice). x is a vector representing states and u is an input vector.
 - You can set the whole parameters related to discrete sampling period and you wil get time response of system in Matlab script and simulink versions


For more detailed information about us,   

  http://www.phinitelab.com/
  https://www.udemy.com/user/phinite-academy/
  
