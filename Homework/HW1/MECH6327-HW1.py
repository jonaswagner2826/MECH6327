# -*- coding: utf-8 -*-
"""
Created on Thu Jan 28 23:30:58 2021

@author: Jonas
"""


import numpy as np
import matplotlib.pyplot as plt


# Problem 2.3
x = np.linspace(-1,1)
y1 = 1 - abs(x)
y2 = np.sqrt(1 - x**2)
y3 = 1


fig,axs = plt.subplots(1,3,sharex=True,figsize=(10,10))

axs[0].plot([x,x],[y1,-y1])
axs[0].set_box_aspect(1)
axs[0].set_title('p=1')

axs[1].plot([x,x],[y2,-y2])
axs[1].set_box_aspect(1)
axs[1].set_title('p=2')

axs[2].plot([x,x],[y3,-y3])
axs[2].set_box_aspect(1)
axs[2].set_title('p=infinity')