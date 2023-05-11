#%% Introduction
import numpy as np
from control.matlab import *

a = np.matrix([[0,        1,        0],
                [-0.0071,-0.111, 0.12],
                [0,       0.07,  -0.3]])
b = np.matrix([ [ 0.   ],
                [-0.095],
                [ 0.072]])
c = np.matrix(  [1, 0, 0])
d = np.matrix(  [0])

sys = ss(a, b, c, d)

newc = np.matrix([[1.0, 0,  0],  # theta
                 [0,   1,  0],  # q
                 [0,   0,  1],  # aplha
                 [1,   0, -1]])  # gamma = theta - alpha
newd = np.zeros((4,1))
sys2 = ss(a, b, newc, newd)

#%% 06.1 intro 1
import numpy as np

# parameter values
m = 4.5
b = 9
k = 60

# matrices
A = np.matrix([[0,1],
               [-k/m,-b/m]])
B = np.matrix([[0],
               [1/m]])
C = np.matrix([[1,0],
               [0,1],
               [-k/m,-b/m]])
D = np.matrix([[0],
               [0],
               [1/m]])

sys = ss(A, B, C, D)

#%% 06.1 intro 2
import numpy as np
from control.matlab import *
import matplotlib.pyplot as plt
# parameter values
m = 4.5
b = 9
k = 60
# matrices
A = np.matrix([[ -b/m, -k/m], [ 1, 0]])
B = np.matrix([[ 1/m], [ 0]])
C = np.matrix([[ 0, 1]])
D = np.matrix([[ 0]])
# system
sys = ss(A, B, C, D)

x0 = np.matrix([[1], [0]])
t = np.linspace(0, 10, 10000)

y , t = initial(sys,X0=x0,T=t)
plt.plot(t,y)
plt.grid()
plt.show()

print(t[np.logical_and(y>0, t>1.5)][0])

#%% 06.1 intro 3 sine response
import numpy as np
from control.matlab import *
import matplotlib.pyplot as plt
# parameter values
m = 4.5
b = 9
k = 60

w1 = 3.5118846
w2 = 3.3665016
# matrices
A = np.matrix([[ -b/m, -k/m], [ 1, 0]])
B = np.matrix([[ 1/m], [ 0]])
C = np.matrix([[ 0, 1]])
D = np.matrix([[ 0]])
# system
sys = ss(A, B, C, D)

dt = 0.01
t = np.arange(0, 20+dt, dt)
x0 = np.matrix([[1], [0]])

sin1 = np.sin(w1*t)
sin2 = np.sin(w2*t)

y1, t, _ = lsim(sys, U=sin1,T=t )
y2, t, _ = lsim(sys, U=sin2,T=t )

print(max(y1),max(y2))