#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec  7 20:30:24 2020

@author: christopher
"""
import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt
import planeParam as plane
import control
import control.matlab
import math

# get state space matrix
A_lat = plane.A_lat
B_lat = plane.B_lat
C_lat = plane.C_lat
D_lat = plane.D_lat

A_long = plane.A_long
B_long = plane.B_long
C_long = plane.C_long
D_long = plane.D_long

# calculate the eigenvalues
E_lat = LA.eigvals(A_lat)

# create open loop system
ss_lat = control.StateSpace(A_lat, B_lat, C_lat, D_lat)
ss_long = control.StateSpace(A_long, B_long, C_long, D_long)

# simulation time properties
t_end = 10
t_step = 0.001
length = int(t_end/t_step)       #The number of time steps over the time interval
t_array = np.linspace(0, t_end, length)   #The time vector that is passed into the ode

# controllability 
Co_lat = control.ctrb(A_lat, B_lat)
Unco_lat = len(A_lat) - LA.matrix_rank(Co_lat)
print('Uncontrollable States: ' + str(Unco_lat))

# observability
Ob_lat = control.obsv(A_lat, C_lat)
Unob_lat = len(A_lat) - LA.matrix_rank(Ob_lat)
print('Unobservable States: ' + str(Unob_lat))

# get lateral response to aileron step
x0 = np.array([[0], [0], [0], [0]])
u = np.transpose(math.pi/18*np.array([np.ones((length)), np.zeros((length))]))
y, t, xout = control.matlab.lsim(ss_lat, u, t_array, x0)

fig, axs = plt.subplots(4)
plt.tight_layout(pad=1.14, h_pad=None, w_pad=None, rect=None)
plt.xlabel('Time [s]')
fig.suptitle('Response to Aileron Step')
axs[0].plot(t, y[:,0]*180/math.pi)
axs[0].set_ylabel('[deg]')
axs[0].set_title('Side-Slip')
axs[1].plot(t, y[:,1]*180/math.pi)
axs[1].set_ylabel('[deg/s]')
axs[1].set_title('Yaw-Rate')
axs[2].plot(t, y[:,2]*180/math.pi)
axs[2].set_ylabel('[deg/s]')
axs[2].set_title('Roll Rate')
axs[3].plot(t, y[:,3]*180/math.pi)
axs[3].set_ylabel('[deg]')
axs[3].set_title('Bank Angle')

# get lateral response to rudder step
#t, y = control.step_response(ss_lat, input=1)
x0 = np.array([[0], [0], [0], [0]])
u = np.transpose(np.array([np.zeros((length)), math.pi/18*np.ones((length))]))
y, t, xout = control.matlab.lsim(ss_lat, u, t_array, x0)

fig, axs = plt.subplots(4)
plt.tight_layout(pad=1.04, h_pad=None, w_pad=None, rect=None)
plt.xlabel('Time [s]')
fig.suptitle('Response to Rudder Step')
axs[0].plot(t, y[:,0]*180/math.pi)
axs[0].set_ylabel('[deg]')
axs[0].set_title('Side-Slip')
axs[1].plot(t, y[:,1]*180/math.pi)
axs[1].set_ylabel('[deg/s]')
axs[1].set_title('Yaw-Rate')
axs[2].plot(t, y[:,2]*180/math.pi)
axs[2].set_ylabel('[deg/s]')
axs[2].set_title('Roll Rate')
axs[3].plot(t, y[:,3]*180/math.pi)
axs[3].set_ylabel('[deg]')
axs[3].set_title('Bank Angle')

'''
# get response to throttle step
t, y = control.step_response(ss_long, input=0)

fig, axs = plt.subplots(5)
fig.suptitle('Response to Throttle Step')
axs[0].plot(t, y[0])
axs[0].set_ylabel('Amplitude')
axs[0].set_title('Relative Velocity (V/V0)')
axs[1].plot(t, y[1])
axs[1].set_ylabel('Amplitude')
axs[1].set_title('Climb Angle')
axs[2].plot(t, y[2])
axs[2].set_ylabel('Amplitude')
axs[2].set_title('Angle of Attack')
axs[3].plot(t, y[3])
axs[3].set_ylabel('Amplitude')
axs[3].set_title('Pitch Rate')
axs[4].plot(t, y[4])
axs[4].set_ylabel('Amplitude')
axs[4].set_title('Height')

# get response to elevator step
t, y = control.step_response(ss_long, input=1)

fig, axs = plt.subplots(5)
fig.suptitle('Response to Elevator Step')
axs[0].plot(t, y[0])
axs[0].set_ylabel('Amplitude')
axs[0].set_title('Relative Velocity (V/V0)')
axs[1].plot(t, y[1])
axs[1].set_ylabel('Amplitude')
axs[1].set_title('Climb Angle')
axs[2].plot(t, y[2])
axs[2].set_ylabel('Amplitude')
axs[2].set_title('Angle of Attack')
axs[3].plot(t, y[3])
axs[3].set_ylabel('Amplitude')
axs[3].set_title('Pitch Rate')
axs[4].plot(t, y[4])
axs[4].set_ylabel('Amplitude')
axs[4].set_title('Height')
'''