# RUN ME
# This file gives the initial condition to the scipy.integrat.odeint function 
# and plots the resulting state outputs at each time step in an animation and
# on a plot that compares the actual output with the reference input

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import planeParam
import simParam
import controllerParam
import planeControllerDynamics as planeCtl
from planeControllerDynamics import planeCnt  
from signalGenerator import signalGen

#Generate the reference signal
length = int((simParam.t_end-simParam.t_start)/simParam.Ts)       #The number of time steps over the time interval
t_array = np.linspace(simParam.t_start, simParam.t_end, length)   #The time vector that is passed into the ode
beta_ref = signalGen(amplitude=0, frequency= 0.001, y_offset=0)
r_ref = signalGen(amplitude=0, frequency= 0.001, y_offset=0)
p_ref = signalGen(amplitude=0, frequency= 0.001, y_offset=0)
phi_ref = signalGen(amplitude=0, frequency= 0.08, y_offset=0)
ctl_ref = np.array([[beta_ref.square], [r_ref.square], [p_ref.square], [phi_ref.square]])
'''
beta_ref = signalGen(amplitude=math.pi/30, frequency= 0.1, y_offset=0).square
r_ref = signalGen(amplitude=0, frequency= 0.001, y_offset=0).square
p_ref = signalGen(amplitude=0, frequency= 0.001, y_offset=0).square
phi_ref = signalGen(amplitude=0, frequency= 0.08, y_offset=0).square
ctl_ref = np.zeros((length, 4))
for t in range(0, length):
    ctl_ref[t, 0] = beta_ref(t_array[t])[0]
    ctl_ref[t, 1] = r_ref(t_array[t])[0]
    ctl_ref[t, 2] = p_ref(t_array[t])[0]
    ctl_ref[t, 3] = phi_ref(t_array[t])[0] 
'''

#Initialize and rename for convenience
ctrl = planeCnt(planeParam=planeParam, simParam=simParam, controllerParam=controllerParam, ref=ctl_ref)

plt.close('all')
#animation = pendulumAn()

#performs Runga-Kutta to get state values at each time step (equivalent of ode45 in MATLAB)
STATES = odeint(ctrl.planefunc, [simParam.beta_i, simParam.p_i, simParam.r_i, simParam.phi_i], t_array) #Solve non-linear dynamics

# plot relevant states, reference signals, control inputs
fig, axs = plt.subplots(6, 1, sharex=True)
fig.suptitle('Lateral CL response')
plt.tight_layout(pad=1.04, h_pad=None, w_pad=None, rect=None)
plt.xlabel('Time [s]')
axs[0].plot(t_array, np.multiply(STATES[:,0], 180/math.pi), label='beta')
axs[0].plot(np.asarray(planeCtl.ref_sig)[:,4], np.multiply(np.asarray(planeCtl.ref_sig)[:,0], 180/math.pi), label='ref')
axs[0].legend(loc="upper right")
axs[0].set_ylabel('[deg]')
axs[0].set_title('Side-Slip')
axs[1].plot(t_array, np.multiply(STATES[:,1], 180/math.pi), label='r')
axs[1].plot(np.asarray(planeCtl.ref_sig)[:,4], np.multiply(np.asarray(planeCtl.ref_sig)[:,1], 180/math.pi), label='ref')
axs[1].legend(loc="upper right")
axs[1].set_ylabel('[deg/s]')
axs[1].set_title('Yaw Rate')
axs[2].plot(t_array, np.multiply(STATES[:,2], 180/math.pi), label='p')
axs[2].plot(np.asarray(planeCtl.ref_sig)[:,4], np.multiply(np.asarray(planeCtl.ref_sig)[:,2], 180/math.pi), label='ref')
axs[2].legend(loc="upper right")
axs[2].set_ylabel('[deg/s]')
axs[2].set_title('Roll Rate')
axs[3].plot(t_array, np.multiply(STATES[:,3], 180/math.pi), label='phi')
axs[3].plot(np.asarray(planeCtl.ref_sig)[:,4], np.multiply(np.asarray(planeCtl.ref_sig)[:,3], 180/math.pi), label='ref')
axs[3].legend(loc="upper right")
axs[3].set_ylabel('[deg]')
axs[3].set_title('Bank Angle')
axs[4].plot(np.asarray(planeCtl.control_inputs)[:,2], np.multiply(180/math.pi, np.asarray(planeCtl.control_inputs)[:,0]))
axs[4].set_ylabel('[deg]')
axs[4].set_title('Rudder Deflection')
axs[5].plot(np.asarray(planeCtl.control_inputs)[:,2], np.multiply(180/math.pi, np.asarray(planeCtl.control_inputs)[:,1]))
axs[5].set_ylabel('[deg]')
axs[5].set_title('Aileron Deflection')

'''
#animation
i = 0
reference = np.zeros((length,1))
while i < len(t_array):  
    #calculate the reference value
    t_curr = t_array[i]                 #Current time step
    z_ref = ctrl.zref(t_curr)           #Get cart z location
    reference[i:(i+100)] = z_ref        #Which reference value is used

    #update animation and data plots
    animation.drawPendulum(STATES[i,:]) #Animate
    plt.pause(0.001)                    #Pause while plot updates
    i = i + 100                         #speeds up the simulation by not plotting all the points
 
# Plot how closely the actual performance of the pendulum on a cart matched the desired performance
dataPlot = plotData()                       #initializes plot
dataPlot.Plot(t_array, reference, STATES)   #plots the data
'''