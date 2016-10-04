#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author:Atsushi Sakai

import matplotlib.pyplot as plt
import numpy as np
import math 
import pandas as pd

def MotionModel(x,u,dt):
    x[0]=x[0]+u[0]*dt*math.cos(x[2])
    x[1]=x[1]+u[0]*dt*math.sin(x[2])
    x[2]=x[2]+u[1]*dt
    return x

def AddInputNoise(u,Q):
    un=u[:]
    un[0]=un[0]+np.random.randn()*Q[0]
    un[1]=un[1]+np.random.randn()*Q[1]
    return un

def Observation(xTrue,R,time,zdt):
    z=[0,0]

    if abs(time%zdt) > 0.1:
        return z

    z[0]=xTrue[0]
    z[1]=xTrue[1]

    return z

    

xTrue=[0,0,0]# state x,y,yaw
x=[0,0,0]# state x,y,yaw
u=[1.0,0.1]# input v[m/s],omega[rad/s]
Q=[0.5,0.1] #input noise
R=[0.1,0.1] #input noise
z=[0,0]# state x,y,yaw
dt=0.1  #[s]
zdt=4.0  #[s]
time=0.0
SimTime=50.0

true_x=[]
true_y=[]
true_yaw=[]
odo_x   = []
odo_y   = []
odo_yaw = []
z_x   = []
z_y   = []

plt.grid(True)
plt.axis("equal")

while time<SimTime:
    time+=dt

    xTrue=MotionModel(xTrue,u,dt)
    un=AddInputNoise(u,Q)
    x=MotionModel(x,un,dt)

    z=Observation(xTrue,R,time,zdt)

    #Show graph
    #  plt.plot(xTrue[0],xTrue[1],".b");
    #  plt.plot(x[0],x[1],".r");
    #  plt.pause(0.001)

    true_x.append(xTrue[0])
    true_y.append(xTrue[1])
    true_yaw.append(xTrue[2])
    odo_x.append(x[0])
    odo_y.append(x[1])
    odo_yaw.append(x[2])
    z_x.append(z[0])
    z_y.append(z[1])

plt.plot(true_x,true_y,"-b",label="True");
plt.plot(odo_x,odo_y,"-r",label="odometry");
plt.plot(z_x,z_y,"xg",label="GPS");
plt.legend()
plt.show()


#csv
df=pd.DataFrame()
df["true_x"]   = true_x
df["true_y"]   = true_y
df["true_yaw"] = true_yaw
df["odo_x"]    = odo_x
df["odo_y"]    = odo_y
df["odo_yaw"]  = odo_yaw
df["z_x"]    = z_x
df["z_y"]    = z_y

df.to_csv("data.csv")

