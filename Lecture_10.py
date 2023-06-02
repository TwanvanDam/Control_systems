#%% Attitude control, take 1
import control.matlab as cm
import math
import control

s = cm.tf([1, 0], [1])

zeta = 2 / math.sqrt(13)
omega = math.sqrt(13)
Kq = -24
Ttheta2 = 1.4

H = cm.tf(-(Kq*(1 + Ttheta2*s))/(s*(s**2 + 2*zeta*omega*s + omega**2)))

# try to find a gain that gets the real axis poles to -0.4
control.sisotool(H)

#%%  Increase short-period damping Rate feedback
import control.matlab as cm
import math
import control

s = cm.tf([1, 0], [1])

zeta = 2 / math.sqrt(13)
omega = math.sqrt(13)
Kq = -24
Ttheta2 = 1.4

H = cm.tf(-(Kq*(1 + Ttheta2*s))/(s**2 + 2*zeta*omega*s + omega**2))

# try to find a gain that gets a damping coefficient of 0.9
control.sisotool(H)


#%% Getting the new system
import control.matlab as cm
import math

s = cm.tf([1, 0], [1])

Vtas = 160
zeta = 2 / math.sqrt(13)
omega = math.sqrt(13)
Kq = -24
Ttheta2 = 1.4

Hq = cm.tf((Kq*(1 + Ttheta2*s))/(s**2 + 2*zeta*omega*s + omega**2))
Htheta = cm.tf(Hq/s)
Hgamma = cm.tf(Hq/(s*(1+Ttheta2*s)))
Hh = cm.tf((Hq*Vtas)/(s**2*(1+Ttheta2*s)))

Hall = cm.tf([[Hq.num[0][0]],
           [Htheta.num[0][0]],
           [Hgamma.num[0][0]],
           [Hh.num[0][0]]],
          [[Hq.den[0][0]],
            [Htheta.den[0][0]],
            [Hgamma.den[0][0]],
            [Hh.den[0][0]]])

sys1 = cm.minreal(cm.ss(Hall))
print(sys1)


# Model answer --> minreal is not needed for the right answer
s = cm.tf([1, 0], [1])
Hq = Kq*(1+Ttheta2*s)/(s**2 + 2*zeta*omega*s + omega**2)
Htheta = Hq/s
Hgamma = Kq/(s*(s**2 + 2*zeta*omega*s + omega**2))
Hh = Vtas*Hgamma/s

Hall2 = cm.tf([[Hq.num[0][0]],
           [Htheta.num[0][0]],
           [Hgamma.num[0][0]],
           [Hh.num[0][0]]],
          [[Hq.den[0][0]],
            [Htheta.den[0][0]],
            [Hgamma.den[0][0]],
            [Hh.den[0][0]]])
sys2 = cm.ss(Hall2)
print(sys2)

#%% Getting the new system
import control.matlab as cm
import math
import numpy as np
import matplotlib.pyplot as plt

s = cm.tf([1, 0], [1])

Vtas = 160
zeta = 2 / math.sqrt(13)
omega = math.sqrt(13)
Kq = -24
Kr = -0.089
Ttheta2 = 1.4

Hq = cm.tf((Kq*(1 + Ttheta2*s))/(s**2 + 2*zeta*omega*s + omega**2))
Htheta = cm.tf(Hq/s)
Hgamma = cm.tf(Hq/(s*(1+Ttheta2*s)))
Hh = cm.tf((Hq*Vtas)/(s**2*(1+Ttheta2*s)))

Hall = cm.tf([[Hq.num[0][0]],
           [Htheta.num[0][0]],
           [Hgamma.num[0][0]],
           [Hh.num[0][0]]],
          [[Hq.den[0][0]],
            [Htheta.den[0][0]],
            [Hgamma.den[0][0]],
            [Hh.den[0][0]]])

sys1 = cm.ss(Hall)

K = np.matrix([Kr , 0 , 0 , 0])
sys2 = cm.feedback(sys1,K)

y1 , t1 = cm.step(sys1,4)
y2, t2 = cm.step(sys2,4)

plt.plot(t1,y1[:,0])
plt.plot(t2,y2[:,0])
plt.show()

print(sys2)

#%%  Tuning the attittude control
import control.matlab as cm
import math
import numpy as np
import matplotlib.pyplot as plt

s = cm.tf([1, 0], [1])

Vtas = 160
zeta = 2 / math.sqrt(13)
omega = math.sqrt(13)
Kq = -24
Kr = -0.089
Ttheta2 = 1.4

Hq = cm.tf((Kq*(1 + Ttheta2*s))/(s**2 + 2*zeta*omega*s + omega**2))
Htheta = cm.tf(Hq/s)
Hgamma = cm.tf(Hq/(s*(1+Ttheta2*s)))
Hh = cm.tf((Hq*Vtas)/(s**2*(1+Ttheta2*s)))

Hall = cm.tf([[Hq.num[0][0]],
           [Htheta.num[0][0]],
           [Hgamma.num[0][0]],
           [Hh.num[0][0]]],
          [[Hq.den[0][0]],
            [Htheta.den[0][0]],
            [Hgamma.den[0][0]],
            [Hh.den[0][0]]])

sys1 = cm.ss(Hall)

K = np.matrix([Kr , 0 , 0 , 0])
sys2 = cm.feedback(sys1,K)
# until here all the same as previous
Hth =  cm.tf(sys2[1,:])
Ktheta = -0.47365708
control.sisotool(-cm.tf(s/s*Hth),initial_gain=-Ktheta,xlim_rlocus=[-1.5,0.5],ylim_rlocus=[-10,10]) #tool is very slow and weird

Hth_closed = Hth.feedback(Ktheta)
print(cm.damp(Hth_closed))

#%% Making the θ feedback controller
import control.matlab as cm
import math
import numpy as np
import matplotlib.pyplot as plt

s = cm.tf([1, 0], [1])

Vtas = 160
zeta = 2 / math.sqrt(13)
omega = math.sqrt(13)
Kq = -24
Kr = -0.089
Ttheta2 = 1.4
Ktheta = -0.47365708

h = cm.tf(1/(1+2.5*s))

Hq = cm.tf((Kq*(1 + Ttheta2*s))/(s**2 + 2*zeta*omega*s + omega**2))
Htheta = cm.tf(Hq/s)
Hgamma = cm.tf(Hq/(s*(1+Ttheta2*s)))
Hh = cm.tf((Hq*Vtas)/(s**2*(1+Ttheta2*s)))

Hall = cm.tf([[Hq.num[0][0]],
           [Htheta.num[0][0]],
           [Hgamma.num[0][0]],
           [Hh.num[0][0]]],
          [[Hq.den[0][0]],
            [Htheta.den[0][0]],
            [Hgamma.den[0][0]],
            [Hh.den[0][0]]])

sys1 = cm.ss(Hall)

K = np.matrix([Kr , 0 , 0 , 0])
sys2 = cm.feedback(sys1,K)

# make new system 3
sys3 = (sys2*Ktheta).feedback([0, 1, 0, 0]) # not sure if it is correct answers give very different matrices maybe equivalent

t = np.arange(0, 20, 0.01)
y1, t = cm.step(sys3, t, output=1)

# second system, with time constant of 2.5s
hcomp = 1/(1+2.5*s)
yc, t = cm.step(hcomp, t)

# settling times
print(t[(y1 < 0.95) + (y1 > 1.05)][-1])
print(t[(yc < 0.95) + (yc > 1.05)][-1])

#%% Attitude hold controller as a basis
import control.matlab as cm
import math
import numpy as np
import matplotlib.pyplot as plt

s = cm.tf([1, 0], [1])

Vtas = 160
zeta = 2 / math.sqrt(13)
omega = math.sqrt(13)
Kq = -24
Kr = -0.089
Ttheta2 = 1.4
Ktheta = -0.47365708

Hq = cm.tf((Kq*(1 + Ttheta2*s))/(s**2 + 2*zeta*omega*s + omega**2))
Htheta = cm.tf(Hq/s)
Hgamma = cm.tf(Hq/(s*(1+Ttheta2*s)))
Hh = cm.tf((Hq*Vtas)/(s**2*(1+Ttheta2*s)))

Hall = cm.tf([[Hq.num[0][0]],[Htheta.num[0][0]],[Hgamma.num[0][0]],[Hh.num[0][0]]],[[Hq.den[0][0]], [Htheta.den[0][0]],[Hgamma.den[0][0]],[Hh.den[0][0]]])
sys1 = cm.ss(Hall)

K = np.matrix([Kr , 0 , 0 , 0])
sys2 = cm.feedback(sys1,K)

# make new system 3
sys3 = (sys2*Ktheta).feedback([0, 1, 0, 0]) # not sure if it is correct answers give very different matrices maybe equivalent

t = np.arange(0, 20, 0.01)
y, t = cm.step(sys3, t)

y3 = y[:,2]

plt.plot(t,y[:,2])
plt.show()
print(t[((y3 < 0.95) + (y3 > 1.05))[:,0]][-1])

#%% Altitude controller
import control.matlab as cm
import math
import numpy as np
import matplotlib.pyplot as plt

s = cm.tf([1, 0], [1])

Vtas = 160
zeta = 2 / math.sqrt(13)
omega = math.sqrt(13)
Kq = -24
Kr = -0.089
Ttheta2 = 1.4
Ktheta = -0.47365708

Hq = cm.tf((Kq*(1 + Ttheta2*s))/(s**2 + 2*zeta*omega*s + omega**2))
Htheta = cm.tf(Hq/s)
Hgamma = cm.tf(Hq/(s*(1+Ttheta2*s)))
Hh = cm.tf((Hq*Vtas)/(s**2*(1+Ttheta2*s)))

Hall = cm.tf([[Hq.num[0][0]],[Htheta.num[0][0]],[Hgamma.num[0][0]],[Hh.num[0][0]]],[[Hq.den[0][0]], [Htheta.den[0][0]],[Hgamma.den[0][0]],[Hh.den[0][0]]])
sys1 = cm.ss(Hall)

K = np.matrix([Kr , 0 , 0 , 0])
sys2 = cm.feedback(sys1,K)

# make new system 3
sys3 = (sys2*Ktheta).feedback([0, 1, 0, 0]) # not sure if it is correct answers give very different matrices maybe equivalent

H = cm.tf(np.matrix([[0, 0, 0, 1]])*sys3).minreal()
# the removal process is slightly different here, we just remove the small leading terms of the numerator vector
H.num[0][0] = H.num[0][0][-1:]
#control.sisotool(H)

Kh = 0.00116
print(cm.damp(H.feedback(Kh)))
y,t = cm.step(Kh*sys3,10,output=0)
plt.plot(t,y)
plt.show()
print(t[(y < 0.95*y[-1]) + (y > 1.05*y[-1])][-1]) # actual answer is 10.85