import matplotlib.pyplot as plt
import numpy as np
import time


t = np.arange(0,6,0.01)
# print np.cos(t)

# Lemniscate of Bernoulli
T = 6
t_ = t/T*2*np.pi + np.pi/2
a = 0.03
ratio = 0.8
x = ratio*(a * 2**0.5 * np.cos(t_)) / (np.sin(t_)*np.sin(t_) + 1)
y = (a * 2**0.5 * np.cos(t_) * np.sin(t_)) / (np.sin(t_)*np.sin(t_) + 1)

plt.plot(x,y)
plt.xlim([-0.06, 0.06])
plt.ylim([-0.06, 0.06])
# plt.show()

a = np.array([1])
print a.shape

if a.shape == (0,):
    print "haha"
    print a, type(a), type(a.shape)