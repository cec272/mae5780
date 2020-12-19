# This file contains physical parameters for plane state space equations

import simParam as sim
import math
import numpy as npy

####################################################
#               Aircraft Paramters
####################################################
# fuselage properties
Lf = 1.492                      # length of fuselage [m]
Rf = .322                       # radius of fuselage [m]
Vf = math.pi*math.pow(Rf,2)*Lf  # cylindrical volumetric approximationi of fuselage [m^3]
Knb =  3                        # impact of fuselage on horizontal stabilizer, typ.=3 [nondimm]
Knr = 1.5                       # impace of fuselage on vertical stabilizer, typ.=1.5 [nondimm]
Xf = 0.931                      # distance to the aerodynmaic center of wing [m]
Xg = 0.95                       # distance to the center of gravity [m]
Xfe = .106                      # distance to the aerodynamic center of empennage [m]
Xfvs = .106                     # distance to the aerodynamic center of vertical stabilizer [m]
d = Xf-Xfe                      # distance between aerodynamic centers, wing-empennage [m]
Zvs = .211                      # vertical distance between CG and aerodynamic center of verticall tail [m] 

# wing properties
b = 1.517               # wingspan [m]
ell = 0.203             # wing mean aerodynamic chrod [m]
S = b*ell               # wing surface area [m^2]
AR =  math.pow(b,2)/S   # wing aspect ratio [nondimm]
Gamma = 0.043369        # dihedral angle of wings [rad]
Lambda = 1              # taper ratio of wings [nondim]
alpha0 = -0.0087266     # zero-lift angle of attack [rad]
dw = 0                  # angle between fuselage axis and wing chord line [rad]
e = .70                 # wing efficiency factor, typ.=0.7 for rectangular or =1 for eliptical wings [nondimm]

# horizontal stabilizer properties
be = 0.305                  # empennage (horz stabilizer) span [m]
elle = 0.128                # empennage (horz stabilizer) mean aerodynamic chord [m]
Se = be*elle                # empennage (horz stabilizer) surface area [m^2]
ARe = math.pow(be,2)/Se     # empennage (horz stabilizer) aspect ratio [nondimm]
etae = 0.9                  # empennage (horz stabilizer) dynamic pressure  ratio, typ.=0.9 [nondimm]
de = 0                      # angle between fuselage axis and empennage chord line [rad]
Ve = (Se*(Xg-Xfe))/(S*ell)  # empennage (horz stabilizer) volume coefficient, 0.3<typ.<0.6 [nondimm]

# vertical stabilizer properties
bvs = 0.254                    # vertical stabilizer span [m] 
ellvs = 0.116                  # vertical stabilizer mean aerodynamic chord [m] 
Svs =  bvs*ellvs               # vertical stabilizer surface area [m^2]
ARvs = math.pow(bvs,2)/Svs     # vertical stabilizer aspect ratio [nondimm]
etavs = 0.95                   # vertical stabillizer dynamic pressure ratio, typ.>0.9 [nondimm] 
Vvs = (Svs*(Xg-Xfvs))/(S*ell)  # vertical stabilizer voolume coefficient, 0.02<typ.<0.05 

# surface control parameters
bdm = 0.305                                         # elevator span [m]
elldm = 0.025                                       # elevator mean aerodynamic chord [m]
taudm = 3.1174*math.pow((elldm/elle),3)-4.6286*math.pow((elldm/elle),2)+2.8661*(elldm/elle)     # elevator angle of attack effectiveness parameter [nondimm]
dm_range = [-25*math.pi/180, 25*math.pi/180]        # min and max deflection angle of elevator [rad, rad]
bdl = 0.423                                         # aileron span [m]
elldl = 0.051                                       # aileron mean aerodynamic chord [m]
Ydli = 0.064                                        # distance between CG and close ledge of aileron [m]
Ydl0 = 0.487                                        # distance between CG and far ledge of aileron [m]
dl_range = [-25*math.pi/180, 25*math.pi/180]        # min and max deflection angle of aileron [rad, rad]
taudl = 3.1174*math.pow((elldl/ell),3)-4.6286*math.pow((elldl/ell),2)+2.8661*(elldl/ell)        # aileron angle of attack effectiveness parameter [nondimm]
Kdl = -.36                                          # emphirical factor from Roskam, Fig 11.3 [nondimm]
bdn = 0.254                                         # rudder span [m]
elldn = 0.025                                       # rudder mean aerodynamic chord [m]
taudn = 3.1174*math.pow((elldn/ellvs),3)-4.6286*math.pow((elldn/ellvs),2)+2.8661*(elldn/ellvs)  # rudder angle of attack effectiveness parameter [nondimm]
dn_range = [-30*math.pi/180, 30*math.pi/180]        # min and max deflection angle of rudder [rad, rad]

#  masses and  moments  of inertia
m = 2.258                   # total mass [kg]
A = 0.056                   # moment of inertia, Ixx [kg*m^2]
B = 0.504                   # moment of inertia, Iyy [kg*m^2]
C = 0.529                   # moment of inertia, Izz [kg*m^2]
D = 0                       # product of inertia, Ixy [kg*m^2]
E = 0                       # product of inertia, Ixz [kg*m^2]
F = 0                       # product of inertia, Iyz [kg*m^2]
I = npy.array([[A, D, E],   # full moment of inertia tensor [kg*m^2]
               [D, B, F],
               [E, F, C]])

#  engine parameters
Lambdaf = -1        # velocity exponent in thrust model, typ.=-1 for propeller [nondimm]
Thrust = 0.33       # max thrust of motor [N] 
dx_range = [0, 1]   # min and max thrust setting range [nondimm, nondimm]

####################################################
#                 Environment Paramters
####################################################
# parameters for the world
g = sim.g      # gravitational constant [m/s^2]
h = sim.h      # altitude [m]
rho = sim.rho  # air density calculation [kg/m^3]
T = sim.T      # atmospheric temperature [K]

####################################################
#                Initial Conditions
####################################################
# initial conditions  for the equilibrium  state
alpha = sim.alpha               # angle of attack [rad]
theta = sim.theta               # pitch [rad]
V = sim.V                       # velocity [m/s]
Mach = V/(math.sqrt(1.4*287*T)) # mach number [nondimm]

####################################################
#               Aircraft Coefficients
####################################################
# lift coefficient
Czavs = (2*math.pi)/(1+2*ARvs)          # vertical stabilizer lift curve slope [nondimm]
Czae = (2*math.pi)/(1+2*ARe)            # horz. stabilizer lift curve slope [nondimm]
Czaw = (2*math.pi)/(1+2/AR)             # wing lift curve slope [nondimm]
Cza = Czaw + Se/S*Czae                  # total lift curve slope (wing+horz) [nondimm]
Czdm = Czae*etae*Se/S*bdm/be*taudm      # influence of elevators [nondimm]
Czw = Czaw*(alpha+dw-alpha0)            # contribution of wing to Cz [nondimm]
Epsilon = (2*Czw)/(math.pi*AR)          # reduction of empennage AoA due to wing flow [rad]
Cze = Czae*(alpha+de-Epsilon-alpha0)    # contribution of horz. stabilizer to Cz [nondimm]
Cz = Czw + Se/S*Cze                     # total lift coefficient [nondimm]

# drag coefficient
Cx0 = 0.012                                     # drag coefficient at zero lift (skin friction+form [nondimm]           
Cx = Cx0 + 1/(math.pi*AR*e)*math.pow(Cz,2)      # total drag coefficeint (skin friction+form+induced) [nondimm]
f = Cz/Cx                                       # lift to drag ratio [nondimm]

# moment coefficient
Cm0 = 0                             # zero-lift moment coeff, typ.<0 for most aircraft, or =0 for symmetric airfoil [nondimm]
Cmaf = 2*Vf/(S*ell)                 # contribution of fuselage to Cma [nondimm]
Cma = (Xf-Xg)/ell*Cza + Cmaf        # influence of AoA, typ.<0 for static stability [nondimm]
Cmq = -2*etae*(Xfe-Xg)/ell*Ve*Czae  # influence of pitch rate
Cmdm = -Czae*etae*Ve*bdm/be*taudm   # influence of elevators, -4<typ.<-0.2 [nondimm]

# lateral force coefficients
CYb = -(2*math.pi)/(1+2/ARvs)*Svs/S     # influence of slide-slip angle (b), typ.<0 [nondimm]
CYp = 0                                 # influence of roll rate (p), typ.=0 [nondimm]
CYr = 0                                 # influence of yaw rate  (r), typ.=0 [nondimm]
CYdl = 0                                # influence of ailerons (dl), typ.=0 [nondimm]
CYdn = Czavs*etavs*taudn*bdn/bvs*Svs/S  # influence of rudder (dn), typ.>0 [nondimm]

# roll moment coefficients
Clb = -b/ell*(1+2*Lambda)/(1+Lambda)*Cza/6*Gamma                            # influence of slide-slip  (b), typ.<0 [nondimm]
Clp = -math.pi/8*math.pow(AR,3)/(4+AR)                                      # influence of  roll rate (p), typ.<0  [nondimm]
Clr = 1/16*(1+4*Lambda)/(1+Lambda)*math.pow(AR,2)*Cz                        # influence of yaw rate (r), typ.>0 [nondimm]
Cldl = ((2*Czaw*taudl*ell)/(S*b))*(math.pow(Ydl0,2)/2-math.pow(Ydli,2)/2)   # influence of ailerons (dl), typ.<0 [nondimm]                  
Cldn = CYdn*(Zvs*math.cos(alpha)-(Xg-Xfvs)*math.sin(alpha))/b               # influence of rudder (dn), typ.>0 [nondimm]

# yaw moment coefficients
Cnb =  -1/Knb*d/ell*CYb                             # influence of slide-slip (b), typ.>0 [nondimm]
Cnp = 8/(3*math.pi)*bvs/b*Vvs*Czavs                 # influence of  roll rate (p), typ.=0 [nondimm]
Cnr = math.pow((d/ell),2)*(1/math.pow(Knr,2))*CYb    # influence of yaw rate (r), typ.<0 [nondimm]
Cndl = Kdl*Cz*Cldl                                  # influence of ailerons (dl), typ.>0 [nondimm]  
Cndn = -Czavs*Vvs*etavs*taudn*bdn/bvs               # influence of rudder (dn), typ.<0 [nondimm]

####################################################
#              Lateral Matrix  Entries
####################################################
fac  = 0.5*rho*S*math.pow(V,2)
facl = 0.5*rho*S*math.pow(V,2)*ell

Yb = fac/m*CYb
Ydl = fac/m*CYdl
Ydn = fac/m*CYdn

nb = 1/C*facl*Cnb
np = ell/(C*V)*facl*Cnp
nr = ell/(C*V)*facl*Cnr
ndl = 1/C*facl*Cndl
ndn = 1/C*facl*Cndn

lb = 1/A*facl*Clb
lp = ell/(A*V)*facl*Clp
lr = ell/(A*V)*facl*Clr
ldl = 1/A*facl*Cldl
ldn = 1/A*facl*Cldn

####################################################
#              Longitudinal Matrix  Entries
####################################################

Au = g*(Lambdaf-2)/(f*V)
Ag = -g/V
Adx = 1/m*Thrust*math.cos(theta)
Adm = 0             # unsure, made=0

Bu = 2*g/V
Ba = 1/(m*V)*1/2*rho*S*math.pow(V,2)*Cza
Bdm = 1/(m*V)*1/2*rho*S*math.pow(V,2)*Czdm

Da = 1/B*1/2*rho*S*ell*math.pow(V,2)*Cma
Dq = 1/B*1/2*rho*S*ell*math.pow(V,2)*Cmq
Ddm = 1/B*1/2*rho*S*ell*math.pow(V,2)*Cmdm

Eg = V
Edx = 0             # unsure, made=0
Edm = 0             # unsure, made=0

####################################################
#               Lateral State Space
####################################################
# xdot = A*x + B*u
# y = C*x + D*u
# x = [beta (side-slip), r (yaw rate), p (roll rate), phi (bank angle)]'
# u = [dn (rudder), dl (aileron)]'

A_lat = npy.array([[Yb/V, -math.cos(alpha), math.sin(alpha), g*math.cos(theta)/V],
                   [nb, nr, np, 0],
                   [lb, lr, lp, 0],
                   [0, math.tan(theta), 1, 0]])

B_lat = npy.array([[Ydl/V, Ydn/V],
                   [ndl, ndn],
                   [ldl, ldn],
                   [0, 0]])

C_lat = npy.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]]) 

D_lat = npy.array([[0, 0],
                   [0, 0],
                   [0, 0],
                   [0, 0]])

####################################################
#            Longitudinal State Space
####################################################
# xdot = A*x + B*u
# y = C*x + D*u
# x = [u (relative vel (v/ve)), gamma (climb angle), alpha (angle of attack), q (pitch rate), h  (height)]'
# u = [dx (throttle), dm (elevator)]' 

A_long = npy.array([[Au, Ag, 0, 0, 0],
                    [Bu, 0, 0, 0, 0],
                    [0, 0, -Ba, 1, 0],
                    [0, 0, Da, Dq, 0],
                    [0, Eg, 0, 0, 0]])
B_long = npy.array([[Adx, Adm],
                   [0, Bdm],
                   [0, Bdm],
                   [0, Ddm],
                   [Edx, Edm]])

C_long = npy.array([[1, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 1]]) 

D_long = npy.zeros((5, 2))

