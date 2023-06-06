#%% initialise
import control.matlab as cm
import numpy as np
import scipy as sp
from control import sisotool

s = cm.tf([1, 0], [1])

#%% creating open-loop system model
Jb = 400
Jp = 1000
k = 10
b = 5

hb1 = 1/(Jb*s)
hb2 = 1/s
hp1 = 1/(Jp*s)
hp2 = 1/s
sat0 = cm.append(cm.ss(hb1), cm.ss(hb2), k, b, cm.ss(hp1), cm.ss(hp2))

Q = np.matrix([[1,-4,-3], # For an explaination see the pdf attached
               [2,1,0],
               [3,2,-6],
               [4,1,-5],
               [5,3,4],
               [6,5,0]])

inputs = np.matrix([1])
outputs = np.matrix([[1],
                    [2],
                    [5],
                    [6]])

sys = cm.connect(sat0,Q,inputs,outputs)
print(sys)

#%% creating open-loop system model
# run previous cell first to create system

import matplotlib.pyplot as plt

y, t = cm.step(sys,100)

plt.plot(t,y[:,0],label="thetab dot")
plt.plot(t,y[:,2],label="thetap dot")
plt.grid()
plt.legend()
plt.show()

print("After 100 seconds, the payload will have a higher rotational speed than the satellite body", y[-1,2]>y[-1,0])
print("After 100 seconds, the payload will have rotated more than the body", y[-1,3]>y[-1,1])

systf = cm.tf(sys)
satb = cm.tf(systf.num[1][0], systf.den[1][0])

cm.damp(satb)
print("What is the absolute value of the largest difference in rotation between the payload and the body",np.max(np.abs(y[:,1]-y[:,3])))

#%% Initial tuning with Bode
KdKP = 50
Kp = 0.24

Hpt = cm.tf(systf.num[3][0], systf.den[3][0])

Gpd = Kp*(1+KdKP*s)

sisotool(Hpt * Gpd)
# cross over is similar to corner frequency, read off from the sisotool

#%% Looking at notch filters
# staircase input
z1 = 0.0
z2 = 0.7
omega = 20*np.pi

t = np.arange(0, 0.5, 0.001)
u = t - np.mod(t, 0.1)

Hno = (1+ 2 *s * z1 / omega + (s/omega)**2)/(1+ 2 *s* z2 / omega + (s/omega)**2)
# time response
y, t, x = cm.lsim(Hno, u, t)
f1 = plt.figure()
plt.plot(t, y)

print(f"At t = 0.11, the staircase input is {y[t==0.11]}")

# bode plot
f2 = plt.figure()
mag, phase, omega = cm.bode(Hno, sp.logspace(1, 2, 500))

# and a Nyquist plot
f3 = plt.figure()
cm.nyquist(Hno, sp.logspace(0, 3, 500))
plt.show()

# what is -10 db in gain?
tendb = 10**(-10./20)

# get these frequencies. Note that the answer gets more precise with a
# sufficiently large logspace vector over a not too wide range
print(omega[mag < tendb][0], omega[mag < tendb][-1])

#%% Open loop with notch filter
z1 = 0.05
z2 = 0.7
omega = 0.19
Hno = (1+ 2 *s * z1 / omega + (s/omega)**2)/(1+ 2 *s* z2 / omega + (s/omega)**2)

KdKP = 50
Kp = 1.63
Hpt = cm.tf(systf.num[3][0], systf.den[3][0])

Gpd = Kp*(1+KdKP*s)

tf_open = cm.tf(Gpd * Hno * Hpt)

sisotool(tf_open)

#%% Final check
tf_closed = tf_open.feedback()
t = np.arange(0,250.1,0.1)
ramp = t/250

y1, t = cm.step(tf_closed,t)
y2, t,x = cm.lsim(tf_closed,t,t)
plt.plot(t,y1,label="step")
plt.plot(t,y2,label="ramp")
plt.show()

cm.bode(tf_closed)
cm.damp(tf_closed)
