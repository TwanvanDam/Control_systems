#%% transfer functions
import control.matlab as c
s = c.tf([1, 0], [1])

K1 = 1

G = K1 * (1+ (0.2/s))
H1 = 5 / (s**2 + 3*s + 5)

# transfer function 1
tf1 = (H1*G).feedback()
print("tf1 num= ", tf1.num[0][0])
print("tf1 den= ", tf1.den[0][0])

# transfer function 2
tf2 = G.feedback(H1)
print("tf2 num= ", tf2.num[0][0])
print("tf2 den= ", tf2.den[0][0])

# transfer function 3
tf3 = H1.feedback(G)
print("tf3 num= ", tf3.num[0][0])
print("tf3 den= ", tf3.den[0][0])

# transfer function 4
tf4 = 1/(1+H1*G)
print("tf4 num= ", tf4.num[0][0])
print("tf4 den= ", tf4.den[0][0])


#%% Signals 1
import numpy as np
import matplotlib.pyplot as plt

dt = 0.2
f0 = 5.05
T0 = 20     # From inspection of the original plot
f0 = 1/T0

t = np.arange(0.0, 20+dt, dt)
u = np.sin(2*np.pi*f0*t)

plt.plot(t, u)                  # original plot
plt.plot(t,np.sin(t*2*np.pi/T0))  # plot to check period of plotted signal
plt.grid()
plt.show()

print("The period is: ", T0)
print("Frequency is : ", f0)

#%% Signals 2
import numpy as np
import matplotlib.pyplot as plt

dt = 0.15
A = 6.5     # ramp amplitude
e = 7.5     # ramp end

t = np.arange(0, 30+dt, dt)
r = np.hstack((A*t[:(int(e/dt)+1)]/e, A*np.ones(t.shape[0]-51)))

plt.plot(t,r)
plt.grid()
plt.show()

print(np.sum(r))

#%% 4.3 time_2
import control.matlab as c
import numpy as np
import matplotlib.pyplot as plt
s = c.tf([1, 0], [1])
m = 2
k = 20
b = 5
dt = 0.01
t = np.arange(0,10.01,0.01)

H = 1/ (m*s**2 + b*s +k )

y1, t = c.step(H, t)       # function returns response and time vector
y2, t = c.impulse(H, t)   # likewise
plt.plot(t, y1, label="step")
plt.plot(t, y2, label="impulse")
plt.legend()
plt.show()

tsettling = t[np.logical_or(y1 > y1[-1]*1.05, y1 < y1[-1]*0.95)][-1]

overshoot = ((max(y1) / y1[-1]) - 1)*100

tdelay = t[y1 >= 0.1*y1[-1]][0]

trise = t[y1 >= 0.9*y1[-1]][0] - tdelay

#%% 4.3 time_3
import control.matlab as c
import matplotlib.pyplot as plt
import numpy as np

s = c.tf([1, 0], [1])

T = np.arange(0,10.01,0.01)
K = [0.5, 1, 1.5, 2, 2.5, 3]
results = []
for k in K:
    H1 = k / (s + 10)
    H2 = 9 / (s*(s + 1.4))
    H_tot = (H1*H2).feedback()
    y, t = c.step(H_tot, T)
    y_end = y[-1]
    results.append(t[(y < y_end*0.80) | (y > y_end*1.20)][-1])
    plt.plot(t,y,label=f'{k}')
plt.legend()
plt.show()




