# Plot the function exp(-d^2/c^2) for different values of c
# where d is the distance between two points on the unit interval

import numpy as np
import math
import matplotlib.pyplot as plt

# Define the function
def f(d,c):
    return np.exp(-d**2/c**2)

# Plot
c = 768
d = np.linspace(0,c,100)
print(1/math.exp(1))
plt.plot(d,f(d,c),label='c=0.1')
plt.show()
