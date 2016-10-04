#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author:Atsushi Sakai

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

x=np.arange(0,10,0.1)
ty=[np.exp(0.3*tx+0.1) for tx in x]
ey=[np.exp(0.280248*tx+0.2698163) for tx in x]
data=pd.read_csv("data.csv")

plt.plot(data["x"],data["y"],'+r')
plt.plot(x,ty,'-b')
plt.plot(x,ey,'-g')
plt.grid(True)
plt.show()

