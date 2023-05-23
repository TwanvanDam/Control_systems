#%% Dominant poles and step response
import control.matlab as cm
import numpy as np
import matplotlib.pyplot as plt

s = cm.tf([1, 0], [1])

G = 6205 / (s * (s**2 + 13*s + 1281))

H_eq = G.feedback()
poles = H_eq.poles()

#H1 is a first order transfer function, its poles must be real (1/tau + s) = 0 for s = -5 --> 1/tau = 5
H1 = - poles[-1] / (-poles[-1] + s)

#remaining poles are part of H1*H2 = Heq --> H2 = Heq / H1
H2 = (H_eq / H1).minreal()

dt = 0.01
t = np.arange(0,2+dt,dt)
y,_ = cm.step(H_eq,t)
y1,_ = cm.step(H1,t)
y2,_ = cm.step(H2,t)

plt.plot(t,y,t,np.abs(y1),t,np.abs(y2))
plt.show()

D1 = np.trapz(np.abs(y-y1),dx=dt)
D2 = np.trapz(np.abs(y-y2),dx=dt)


#%% Explainations
from control.matlab import ss

import numpy as np

a = np.matrix('''0 1 0 ;

-0.0071 -0.111 0.12;

0 0.07 -0.3''')

b = np.matrix(''' 0 ; -0.095; 0.072''')

c = np.matrix(''' 1.0 0 0;

0 1 0;

0 0 1;

1 0 -1''')

d = np.zeros((4, 1))

sys2 = ss(a, b, c, d)
k = np.matrix( '0  -0.67 0  0')
sysclosed = sys2.feedback(k)

import scipy.linalg as la

print (la.eig(sysclosed.A)[0])

print (la.eig(sys2.A)[0])

#%% Yaw response of an aircraft
import numpy as np
import control.matlab as cm
import matplotlib.pyplot as plt

A = np.matrix([[-0.2, 0.06, 0, -1],
               [0, 0, 1, 0],
               [-17, 0, -3.8, 1],
               [9.4, 0, -0.4, -0.6]])
B = np.matrix([[-0.01, 0.06],
               [0, 0],
               [-32, 5.4],
               [2.6, -7]])

# we only need the rudder input --> delete the first column
B = B[:, 1]

C = np.matrix([0, 0, 0, 1])
D = np.matrix([0])

sys = cm.ss(A, B, C, D)
yaw, t = cm.step(sys, 20)

yaw_min = np.min(yaw)
print(yaw_min)

plt.plot(t, yaw)
plt.show()

#%% Control with state-space systems 1
import numpy as np
import control.matlab as cm

Kd = -0.65

# inputs
A = np.matrix([[-0.2, 0.06, 0, -1],
               [0, 0, 1, 0],
               [-17, 0, -3.8, 1],
               [9.4, 0, -0.4, -0.6]])
B = np.matrix([[-0.01, 0.06],
               [0, 0],
               [-32, 5.4],
               [2.6, -7]])

# outputs
C = np.matrix(np.eye(4))
D = np.matrix(np.zeros((4, 2)))

# feedback
F = np.matrix([[0, 0, 0, 0],
               [0, 0, 0, Kd]])

sys = cm.ss(A, B, C, D)
sys_feedback = sys.feedback(F)

print(sys_feedback)

#%% Control with state-space systems 2
import numpy as np
import control.matlab as cm
import matplotlib.pyplot as plt

Kr = -0.65

# time vector
dt = 0.1
t = np.arange(0.1, 20+dt, dt)

# input signal
u = np.vstack((np.hstack((np.ones((int(1/dt),1)), np.zeros((int(1/dt),1)))),np.zeros((len(t)-int(1/dt),2))))

# original system with only yaw as output
A = np.matrix([[-0.2, 0.06, 0, -1],
               [0, 0, 1, 0],
               [-17, 0, -3.8, 1],
               [9.4, 0, -0.4, -0.6]])
B = np.matrix([[-0.01, 0.06],
               [0, 0],
               [-32, 5.4],
               [2.6, -7]])
C = np.matrix([0, 0, 0, 1])
D = np.matrix([0, 0])

# original system
sys_original = cm.ss(A,B,C,D)

# feedback
F = np.matrix([[0],
              [Kr]])
sys_damper = sys_original.feedback(F)

# wash out system
A_fb = np.mat([[-0.5]])
B_fb = np.mat([[0.5]])
C_fb = np.mat([[0], [-Kr]])
D_fb = np.mat([[0], [Kr]])
K_wash = cm.ss(A_fb, B_fb, C_fb, D_fb)
sys_wash_out = sys_original.feedback(K_wash)

yaw_original = cm.lsim(sys_original,u,t)
yaw_damper = cm.lsim(sys_damper,u,t)
yaw_wash_out = cm.lsim(sys_wash_out,u,t)

plt.plot(t, yaw_original[0], label="original")
plt.plot(t, yaw_damper[0], label="damper")
plt.plot(t, yaw_wash_out[0], label="wash out")
plt.legend()
plt.show()

print(cm.damp(sys_original))
print(cm.damp(sys_damper))

print("Original: ", yaw_original[0][-1])
print("Damper: ",yaw_damper[0][-1])
print("Wash out: ",yaw_wash_out[0][-1])

#%% Constructing system models
from control.matlab import tf, ss, append, connect
import numpy as np
import scipy as sp

# for 2-output, 1-input transfer functions, either enter the coefficient
# matrices directly, as done here
hs = tf([[[1.]],
         [[1, 0]]],
        [[[1, 0, 0]],
         [[1., 0, 0]]])
ssat = ss(hs)

s = tf([1, 0], [1])

# or copy num and den from a transfer function created;
# need tf and derivative here
h2 = 40/(s**2 + 12*s + 40)
hm = tf([[h2.num[0][0]],
         [(h2*s).num[0][0]]],
        [[h2.den[0][0]],
         [(h2*s).den[0][0]]])
ssen = ss(hm)

Ks = 0.5

sys = append(ssat, ssen, Ks)

# connect the inputs and outputs
# outputs 1 and 2: system theta and thetadot
# outputs 3 and 4: sensor theta and thetadot
# output 4: controller
# 1st input: system input, connected to the controller output (5)
# 2nd input: sensor input, connected to satellite theta (1)
# 3rd input: controller in, feedback from sensed position and speed (-3, -4)
Q = np.mat('''1 5 0;
              2 1 0;
              3 -3 -4''')

# remaining input: controller (3)
inputv = [3]
# remaining outputs: controller, theta, thetadot
outputv = [5, 1, 2]
sysc = connect(sys, Q, inputv, outputv)
print (sysc)

hc = (Ks / s**2).feedback(h2 * (1 + s))
print (hc.pole())
print (sp.linalg.eig(sysc.A)[0])

#%% Roll control of an aircraft
import numpy as np
from control.matlab import *

# Laplace variable
s = tf([1, 0], [1])

# autopilot gain
Kap =  1
# actuator
Hact = ss(9/(s**2 + 5*s + 9))
# aircraft
Ka = 4
tau_a = 0.6
Hac = ss(Ka/(1+tau_a*s)/s)

# python-control append will not work when it starts with Kap, need to have
# a state-space system first
sys = append(Hact, Hac, Kap)

# indexing,
# input 1: actuator, connect to autopilot (3)
# input 2: aircraft, connect to actuator (1)
# input 3: autopilot gain, negative feedback from aircraft (-2)
Q = np.mat('''1  3;
              2  1; 
              3 -2''')

inputs = [3]
outputs = [3, 1, 2]

# connect
sysc = connect(sys, Q, inputs, outputs)
print(sysc)


#%% Control of the Moller SkyCar
import numpy as np
from control.matlab import *

s = tf([1, 0], [1])
K = 0.95

ss1 = ss(K * (4*s**2 + 2*s + 1)/(s*(1 + 0.1*s)))

ss2 = ss(1/(s**2*(s**2 + s + 4)))

sys = append(ss1,ss2)
Q = np.matrix([[2, 1],
               [1, -2]])
inputs = np.matrix([1])
outputs = np.matrix([[2],[1]])

sysc = connect(sys,Q,inputs,outputs)
print(sysc)




