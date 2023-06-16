from control.matlab import tf, step, impulse
import numpy as np
import matplotlib.pyplot as plt
from control import sisotool, bode, damp, nyquist_plot,rlocus

s = tf('s')


#%% question 1
H1 = tf((s + 1) / (2*s**4 + 10.4*s**3 + 20.02*s**2 + 40*s + 48.12))
H2 = tf((s + 1) / (2*s**4 + 10*s**3 + 20*s**2 + 40*s + 48))
H3 = tf(-(s-2.5)/((s+1)*(s+3)*(s+2+2j)*(s+2-2j)))

y1 , t1 = step(H1,10)
y2 , t2 = step(H2,10)
y3 , t3 = step(H3,10)

plt.plot(t1,y1,label='1')
plt.plot(t2,y2,label='2')
plt.plot(t3,y3,label='3')
plt.legend()
plt.grid()
plt.show()

#%% question 3
K = 50
H1 = tf(3 / ((s + 12.5) * (s - 1.2)))

sisotool(H1,initial_gain=50)

H_0 =3 / ((12.5) * (-1.2))
final = 9 * (17 * H_0) / (1 + 17 * H_0)
print(final)

#%% question 4
TY = 0.2
KY = 0.015

H = tf(KY*144.5/ ((5*s**2+ 9.5*s)*(TY*s+1)))
sisotool(H)
#bode(H)

#%% question 5
KdKp = 14
KiKp = 0.01

D = tf((1 + (KdKp)*s + (KiKp)*(1/s)))
sat = tf(1.4/s**2)
sens = tf(2/(s+2))

sys_open = tf(D*sat*sens)


sys_closed = tf(0.01165*D*sat).feedback(sens)
rlocus(sys_open)
mag , phase, omega = bode(sys_closed,plot=False)

#%% Sammary
H_OL = tf(0.6*20/(0.5*s**2 + 5 *s))

rlocus(H_OL)

damp(H_OL.feedback())

#%% slides rootlocus
H_OL = tf((s+5)/(s**4 + 3.5*s**3 + 4.25*s**2 - 3.125*s))
sisotool(H_OL)