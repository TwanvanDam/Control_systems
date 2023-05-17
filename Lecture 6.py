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
m = 10
b = 1
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
m = 10
b = 30
k = 30

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
plt.plot(t,y1)

print(max(y1),max(y2))

#%% 6 Conversion
from control.matlab import *
import numpy as np

# can you recognise this one?
a = np.matrix(""" 0       1      0   ;
                 -0.0071 -0.111  0.12;
                  0       0.07  -0.3""")
b = np.matrix(""" 0 ;    -0.095; 0.072""")
c = np.matrix(""" 1       0     0    """)
d = np.matrix(""" 0     """)
sys = ss(a, b, c, d)
h = tf(sys)

# a random transfer function
sys = rss(3, 2, 3)
print(sys)

h = tf(sys)
print (h)
print (h.__class__)

# select one of the transfer functions
h11 = tf(h.num[0][0], h.den[0][0])

# invent a transfer function
s = tf([1, 0], [1])
h = (1 + 2*s)/(s*(s ** 2 + 0.5 *s + 4))
# convert to state-space
sys = ss(h)

# a multi-dimensional transfer function needs to be entered with num and
# den arrays in Python
H = tf( [ [h.num[0][0]], [(h*s).num[0][0]] ],
        [ [h.den[0][0]], [(h*s).den[0][0]] ])
print(H)
#%% 6 Conversion - State-space system to a transfer function
from control.matlab import *
import numpy as np

A = np.matrix( [ [ -1.4,    0,    0  ],
      [ 0.4, -1.2, -5.0],
      [ 0,    1,    0  ] ])
B = np.matrix([ [0.3 ], [0 ], [0 ] ])
C = np.matrix([[0,   1, 0 ],[0.1, 0, 1 ] ] )
D= np.matrix([[0],[0]])

sys = ss(A,B,C,D)
h = tf(sys)

poles = poles(sys)
print("order of numerator of first transfer function:" + str(len(h.num[0][0])-1))
print("order of numerator of second transfer function:" + str(len(h.num[1][0])-1))
print('only real pole of the system' + str(poles[-1]))#only real pole is the last entry of this array

#%% 6 Conversion - Transfer function to a state-space system
import control.matlab as ml

b0 = 0.4
b1 = 0.1
b2 = 0.5
a0 = 2.3
a1 = 6.3
a2 = 3.6
a3 = 1.0

s = ml.tf('s')

#derivative of original output is same as multiplying with s, all the coefficients go one order higher. denominator does not change

num = [[[b2,b1,b0]], [[b2,b1,b0, 0]]]
den = [[[a3, a2, a1,a0]], [[a3, a2, a1,a0]]]
sys1 = ml.tf(num, den)
print(sys1)


ss = ml.ss(sys1)

print(ss)
#%% 6 Conversion - Transfer function to a state-space system
from control.matlab import *
from scipy.linalg import eig

# first create Laplace variable
s = tf([1, 0], [1])
# the coefficients
b0 = 0.4
b1 = 0.1
b2 = 0.5
a0 = 2.3
a1 = 6.3
a2 = 3.6
a3 = 1.0
# create a basic transfer function
h = (b0 + b1*s + b2*s**2)/(a0 + a1*s + a2*s**2 + a3*s**3)
# this combined transfer function has velocity out and the signal itself
H = tf([[h.num[0][0]], [(h*s).num[0][0]]],
       [[h.den[0][0]], [(h*s).den[0][0]]])

# convert to state-space
sys = ss(H)

print(sys)

#%% Block diagram to State space
import numpy as np
K1 = 3.6
K2 = 1.3
K3 = 2.5
K4 = 0.6

A = np.matrix([ [ 0,1,0],[0,-1,1],[-K3*K4,-K2*K4,0]])
B = np.matrix([[0,0],[0,0],[K4,1]])
C = np.matrix([[1,0,K1]])
D = np.matrix([0,0])

sys = ss(A,B,C,D)
print(sys)