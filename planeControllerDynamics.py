# This file contains the scipy.integrate function called in the runSimulation.py file. 

import numpy as np

# keep track of control inputs and reference signal
control_inputs = []
ref_sig = []

class planeCnt:

    def __init__(self, planeParam, simParam, controllerParam, ref):
        
        # state space matricies
        self.A_lat = planeParam.A_lat
        self.B_lat = planeParam.B_lat
        self.C_lat = planeParam.C_lat
        self.D_lat = planeParam.D_lat
        
        # simulation parameters
        self.t_start = simParam.t_start     # Start time of simulation [s]
        self.t_end = simParam.t_end         # End time of simulation [s]
        self.Ts = simParam.Ts               # sample rate of the controller
        self.angle_limit = simParam.angle_limit
        
        # control gain parameters
        if 'Kr_lat' in controllerParam.__dict__.keys():
            self.Kr_lat = controllerParam.Kr_lat
        else:
            self.Kr_lat= 1
            
        if 'K_lat' in controllerParam.__dict__.keys():
            self.K_lat = controllerParam.K_lat
        else:
            self.K_lat= None
        
        # min and max values of control inputs
        self.dm_range = planeParam.dm_range     # elevator angle range
        self.dl_range = planeParam.dl_range     # aileron angle range
        self.dn_range = planeParam.dn_range     # rudder angle range
        self.dx_range = planeParam.dx_range     # thrust setting range
           
        # reference signal parameters
        self.ref=ref
            
    ####################################################
    #               scipy.integrate
    ####################################################
    def planefunc(self,y,t):
        #unpack the state
        beta, r, p, phi = y

        #Get reference inputs from signal generators
        ref = np.array([self.ref[0][0](t), self.ref[1][0](t), self.ref[2][0](t), self.ref[3][0](t)])  # control cart z location
        
        #Add sensor noise to current state values
        curr_state = np.array([[beta], [r], [p], [phi]])   #current state
        
        #Feedback control. If there's no gain it assigns the control to be 0
        if not self.K_lat is None: # a controller has been implemented
            u_des = np.matmul(self.Kr_lat, ref) - np.matmul(self.K_lat, curr_state)  # calculate desired control input

            if u_des[0, 0] < self.dn_range[0]: # ensure control inputs stay within saturation limits
                u_des[0, 0] = self.dn_range[0]
            elif u_des[0, 0] > self.dn_range[1]:
                u_des[0, 0] = self.dn_range[1]
            if u_des[1, 0] < self.dl_range[0]:
                u_des[1, 0] = self.dl_range[0]
            elif u_des[1, 0] > self.dl_range[1]:
                u_des[1, 0] = self.dl_range[1]
            
            self.u = u_des
        else:
            self.u = np.array([[0], [0]])
        
        # append u and r to control outputs for logging
        control_inputs.append([self.u[0, 0], self.u[1, 0], t])    
        ref_sig.append(np.append(ref, [t]))
        
        #calculating state values at the current time step
        ydot = np.matmul(self.A_lat, np.array([[beta], [r], [p], [phi]])) + np.matmul(self.B_lat, self.u)
        dydt = ydot.flatten().tolist()[0] # reshape to [1x4] list
        
        return dydt

    ####################################################
    #              Extra Functions
    ####################################################
    def limitTheta(self, angle):
        if abs(angle) > self.angle_limit:
            rem = (abs(angle) % self.angle_limit)*np.sign(angle)
        elif abs(angle) <= self.angle_limit:
            rem = angle
        return rem
    