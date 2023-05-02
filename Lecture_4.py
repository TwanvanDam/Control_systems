import control.matlab as c
s = c.tf([1, 0], [1])

K1 = 1 
h1 = (1 + 0.4*s)/(s*(s**2+ 3*s +6))
h2 = K1 / (1 + 0.1*s)

frac = h1.feedback(h2)
print("num= ",frac.num[0][0])
print("den= ",frac.den[0][0])


