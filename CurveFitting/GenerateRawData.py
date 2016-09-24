#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author:Atsushi Sakai

import matplotlib.pyplot as plt
import numpy as np

x=np.arange(0,10,0.1)
y=[np.exp(0.3*tx+0.1)+np.random.normal(0.0,0.2) for tx in x]

plt.plot(x,y,'+r')
plt.grid(True)
plt.show()

f=open('data.csv', 'w')
for (x,y) in zip(x,y):
    f.write(str(x)+","+str(y)+"\n")
f.close()


