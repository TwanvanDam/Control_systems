import control.matlab
from control.matlab import tf, step, impulse, ss, append, connect
import numpy as np
import matplotlib.pyplot as plt
from control import sisotool, bode, damp, nyquist_plot,rlocus

s = tf('s')

#%% Question 1
H1 = tf((21*s**2 + 4.4*s + 480.2)/(s**4 + 0.6*s**3 + 404.1*s**2 + 81.61*s +1604))
H2 = tf((30*s**2 + 40*s + 4040)/(s**4 + 2*s**3 + 402*s**2 + 800*s +800))

y1, t1 = step(H1,10)
y2, t2 = step(H2,10)

#plot H1 and H2
plt.plot(t1,y1,t2,y2)
plt.grid()
plt.show()

print("H1(0) =",(480.2)/(1604))
print("position error is :", 1/(1+(480.2)/(1604)))

#%% Question 2
def real_to_dB(num):
    return 20*np.log10(num)

print(real_to_dB(0.3))
#-10 at -180 deg is found for C and magnitude --> inf for omega --> 0 and phase at zero and inf are correct for C

#final value of D is 7 * H(0) / (1 + H(0)) = 7 * H(0) / H(0) = 7

#%% Question 3
A = np.matrix("""-0.9, -3.0, 0; -0.5, -2.5, 0; 0 , 10 , 0""")
B = np.matrix("""0.2; -0.1; 0""")

C = np.matrix([0,0,1])
D = np.matrix([0])

#a)
sys = ss(A,B,C,D)
sys_tf = control.matlab.ss2tf(sys)
print(sys_tf)

#b)
H_new = tf((-0.1*s - 0.0198)/(s**3 + 0.46*s**2 + 0.0401*s))
Kp = -0.1212
#sisotool(-H_new,initial_gain=-Kp)

Kp = -0.2
H_damped = tf(Kp*H_new).feedback()
y, t = step(H_damped,100)
plt.plot(t,y)
plt.show()

settling_time = t[np.logical_or(y >= 1.05*y[-1],y <= 0.95*y[-1])][-1]
print(settling_time)
print("type is 1, one 's' can be factored out of tf")

#%% Question 4
#using append and connect
KY = 0.3
Kphi = 0.5
V = 13
l = 3

H1 = tf(V/s)
H2 = tf(V/(l*s))
H3 = tf(Kphi*s/s)
H4 = tf(KY*s/s)

H_appended = append(H1,H2,H3,H4)

Q = np.matrix([[4,-1,0],
               [3,4,-2],
               [2,3,0],
               [1,2,0]])
inputs = [4,3]
outputs = [1,2,3]

sys = connect(H_appended,Q,inputs,outputs)
print(sys)


#%% Question 5
Kap = 1
Kr = 1
Ta = 0.5

actuator = tf(9/(s**2 + 5*s + 9))
dynamics = tf(Kr / (s*(Ta*s +1)))

#a)
H_a_open = tf(Kap*actuator*dynamics)
H_a_closed = H_a_open.feedback()
print(H_a_closed)

#b)
H_b_closed = tf(s/s).feedback(Kap * H_a_open)
print(H_b_closed)

#c)
sisotool(H_a_open) ; Kap = 0.4

#d) simulate response
H_b_closed = tf(s/s).feedback(Kap * H_a_open)
y1, t1 = step(H_b_closed,100)
print(y1[-1])

#%% real life
Kap = 1
Kr = 1
Ta = 0.5
tphi = 0.8

actuator = tf(9/(s**2 + 5*s + 9))
dynamics = tf(Kr / (s*(Ta*s + 1)))
attitude = tf(1/(tphi*s + 1))
H = tf(Kap*actuator*dynamics)

#e)
H_open_ref = tf(H*attitude)
H_closed_ref = H.feedback(attitude)
H_closed_ref_wrong = H_open_ref.feedback()
print(H_closed_ref_wrong)
print(H_closed_ref) #incorrect grading from mobius

#f)
H_closed_n = tf(s/s).feedback(Kap*H *attitude)
print(H_closed_n)

#g)
sisotool(H_open_ref)
Kap = 0.229

#h)
H_closed_n = tf(s/s).feedback(Kap*H *attitude)
print(H_closed_n)
y2,t2 = step(H_closed_n,100)
print(y2[-1])

#%% plotting for last questoin
plt.plot(t1,y1)
plt.plot(t2,y2)
plt.grid()
plt.show()
#both reject the disturbance the same, the system width the added dynamics just takes longer to get to the final value



