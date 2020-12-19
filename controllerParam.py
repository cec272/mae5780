# This file contains parameters for the LQR controller
from scipy import linalg
import numpy as np
import numpy.linalg as LA
import planeParam as plane
import control

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

####################################################
#                 Controller
####################################################
# LQR
Q_lat = np.matrix([[365, 0, 0, 0],
                   [0, 34, 0, 0],
                   [0, 0, 34, 0],
                   [0, 0, 0, 365]])
R_lat = np.matrix([[8.20, 0],
                   [0, 8.20]])

S_lat = linalg.solve_continuous_are(A_lat, B_lat, Q_lat, R_lat)
K_lat = LA.inv(R_lat).dot(B_lat.transpose().dot(S_lat))

# Find the param value "Kr"
Acl_lat = A_lat - np.matmul(B_lat, K_lat)
sscl_lat = control.StateSpace(Acl_lat, B_lat, C_lat, D_lat)
Kdc_lat = control.dcgain(sscl_lat)
Kr_lat = 0.5*np.transpose(1/Kdc_lat)
Ecl_llat = linalg.eigvals(Acl_lat)