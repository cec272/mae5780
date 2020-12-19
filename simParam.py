# This file contains the parameters for the simulation

import math

####################################################
#                Simulation Paramters
####################################################
# simulation parameters
t_start = 0.0           # Start time of simulation [s]
t_end = 10.0            # End time of simulation [s]
Ts = 0.001              # sample time for simulation [s]
t_plot = 0.5            # Animation update rate [s]
angle_limit = math.pi   # plane cannot go more than pi/2 [rad]

####################################################
#                Initial Conditions
####################################################
# initial conditions  for the equilibrium  state
alpha = 0       # angle of attack [rad]
theta = 0       # pitch [rad]
V = 5           # velocity [m/s]

# lateral control initial solution
beta_i = 0     # side-slip [rad]
p_i = 0         # yaw rate [rad/s]
r_i = 0         # roll rate [rad/s]
phi_i = 10*math.pi/180      # bank angle [rad]

#  initial  controls
dm = 0          # elevator angle [rad]
dx = 0          # thrust control
dn = 0          # rudder angle [rad]
dl = 0          # aileron angle [rad]

####################################################
#                 Environment Paramters
####################################################
# parameters for the environment
g = 9.806                                           # gravitational constant [m/s^2]
h = 10                                              # altitude [m]
press = 101325                                      # air pressure
T = 273                                             # atmospheric temperature [K]
rho = press/(287.058*T)                             # air density calculation [kg/m^3]
