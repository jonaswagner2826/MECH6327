# -*- coding: utf-8 -*-
"""
MECH 6327 - HW 2

Created on Mon Mar  1 23:27:18 2021

@author: Jonas
"""

import numpy as np
import matplotlib.pyplot as plt
import sympy as sp

# Problem 2.7
a = np.array([[2, 4]]).T
b = np.array([[-3, 7]]).T

c = (b - a)
d = (c.T.dot(a + b)) / 2

m = (a+b)/2

# Axis Setup
fig, ax = plt.subplots()
ax.set_aspect('equal', adjustable='datalim')
ax.plot([-5, 5], [0, 0], 'k')
ax.plot([0, 0], [-1, 10], 'k')

# Points a and b
x, y = np.array([a.T, b.T]).T
ax.scatter(x, y)


# Boundary
x = np.linspace(-4,2)
y =  (d - c[0] * x) / c[1]
ax.plot(x,y.T,'r')

plt.savefig('fig/pblm2_7.png')


