import numpy as np
np.random.seed(6)

mat = np.random.randn(1000)
upper = mat<2
lower = mat>-2
count = np.sum(upper*lower)
print(count/10)
print(np.sum((mat<2)&(mat>-2))/10)