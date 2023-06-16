import matplotlib.pyplot as plt
import numpy as np
from control.matlab import *
from control import *

s = tf('s')

#%% Question 1
#approximate H1, with poles and zeros from plot to make the bode plots
H1 = tf(s/((s+2)*(s-1.75j)*(1+1.75j)))
H2 = tf((s+1)/(s**3 + 4*s**2 +5*s + 6))
H3 = tf((s+9)/(2*s**3 + 2*s**2 +11*s))

#a) make bode plots to determine first answer
bode(H1)
bode(H2)
bode(H3)

#b) ss error to unit step impulse and gain = 4 --> 1/(1 + 4 * H2(0)) = 1 / (1 + 4 * 1 / 6 ))

#c) use sisotool to determine closed loop stability use same calculation as b) for the other answers for c)
sisotool(H1)

#%% Question 2
H1 = tf((0.2*s +-0.4) / (s**2))
H2 = tf(1/((s**2 + 0.4*s + 1)*(s+5)))
H3 = tf((-0.5* (s+1)**1)/ ((s+0.3)**2))

y1,t1 = step(H1,5)
y2,t2 = step(H2,5)
y3,t3 = step(H3,5)

#compare tf's
plt.plot(t1,y1,label='1')
plt.plot(t2,y2,label='2')
plt.plot(t3,y3,label='3')
plt.legend()
plt.show()

bode(H1)
bode(H2)
bode(H3)

#%% Question 4
# max overshoot
H1 = tf(36/(s**2+2*s+36))
H2 = tf(25/(s**2+2*s+25))
H3 = tf(9/(s**2+2*s+9))
H4 = tf(16/(s**2+2*s+16))

lst = [H1,H2,H3,H4]

for tf in lst:
    y,t = step(tf,5)
    plt.plot(t,y)
plt.show()

#%% Question 5
R_delta = tf((7.85 * (4*s**2 + 0.6*s + 1)) / ((70*s + 1)*(s**2 + 0.38*s + 1)))

#a) damping ratio for complex poles
damp(R_delta)

#b) new tf
R_delta = tf((7 * (4*s**2 + 0.12*s + 1)) / ((70*s + 1)*(s**2 + 0.9*s + 1)))
damp(R_delta)

#c) root locus for new tf
#sisotool(R_delta,initial_gain=0.8329)
K_low = 0.8329
K_high = 3

#d) similate response for True/False question
y_low, t_low = step(R_delta.feedback(K_low), 50)
y_high, t_high = step(R_delta.feedback(K_high), 50)

#plt.plot(t_low,y_low)
#plt.plot(t_high, y_high)
#plt.show()

#e)
omega = 10
zeta = 0.04
R_delta = tf(R_delta / ((s/omega)**2 + (2*zeta*s/omega) + 1))
#sisotool(R_delta)

#f,g) yaw damper
yaw_damper = tf(1/(1+s))
K = 1.39
sisotool(tf(R_delta*yaw_damper),initial_gain=1.39)
damp(R_delta.feedback(K*yaw_damper))

