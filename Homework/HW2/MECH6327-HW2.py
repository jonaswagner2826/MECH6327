# -*- coding: utf-8 -*-
"""
MECH 6327 - HW 2

Created on Mon Mar  1 23:27:18 2021

@author: Jonas
"""

import numpy as np
import matplotlib.pyplot as plt

# Problem 2.7
a = np.array([[2, 4]]).T
b = np.array([[-3, 7]]).T

c = (a - b)
d = (c.T.dot(a) + c.T.dot(b)) / 2

# Axis Setup
fig, ax = plt.subplots()
ax.plot([-2, 5], [0, 0], 'k')
ax.plot([0, 0], [-4, 8], 'k')

# Points a and b
x, y = np.array([a.T, b.T]).T
ax.scatter(x, y)

# Boundary
X = (np.linspace(-2, 2) * np.ones((2,1))).T
x, y = np.array(- 1 * c.T / X + d).T
ax.plot(x,y)

