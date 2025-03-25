#!/opt/homebrew/Caskroom/miniforge/base/envs/sian/bin/python
import sysv_ipc
import signal
import sys
import os
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D

KEY = 8386
T_SEC=0.1

state=1
while state:
  try:
    shr_memory=sysv_ipc.SharedMemory(KEY)
    state=0
  except:
    pass

shr_val="0"
fuel=0

def vec_angle(v1, v2):
  return np.arccos(np.dot(v1, v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))

def yx_rotation(x, y, vec):
  R_x=np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])
  R_y=np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
  return R_x.dot(R_y.dot(vec))

def encode_data(memory_value):
  li=memory_value.decode("ascii").split('\0')[0]
  li=li.split(':')
  # loc(3), dest(3), velo(3), acc(3), direct(3), t_direct(3), fuel(1)
  loc=np.array(li[0:3], dtype=np.float64); dest=np.array(li[3:6], dtype=np.float64)
  velo=np.array(li[6:9], dtype=np.float64); acc=np.array(li[9:12], dtype=np.float64)
  direct=np.array(li[12:15], dtype=np.float64); t_direct=np.array(li[15:18], dtype=np.float64)
  fuel=float(li[-1])
  return np.vstack([loc, dest, velo, acc, direct, t_direct]), fuel

if __name__=="__main__":
  scale=10
  detection=1000000
  max_reach=200000
  h_dia=0.17; f_dia=0.15
  hei1=3.845; hei2=2.325
  azm=45
  tt=0.01
  
  hei1_vec=np.array([0, 0, hei1])
  hei_mid_vec=np.array([0, 0, (hei1+hei2)/2])
  
  loc=np.zeros([6, 3], dtype=np.float64); fuel=100
  loc[2:-1, 2]=np.ones(3)
  loc[-1, 2]=-1
  
  # ax1
  theta=np.linspace(0, np.pi/2, scale)
  phi=np.linspace(0, np.pi*2, scale)
  # detection
  x1=detection*np.outer(np.cos(phi), np.sin(theta))
  y1=detection*np.outer(np.sin(phi), np.sin(theta))
  z1=detection*np.outer(np.ones(np.size(phi)), np.cos(theta))
  # max_reach
  x2=max_reach*np.outer(np.cos(phi), np.sin(theta))
  y2=max_reach*np.outer(np.sin(phi), np.sin(theta))
  z2=max_reach*np.outer(np.ones(np.size(phi)), np.cos(theta))
  # ax2
  # body part
  x3=np.tile(h_dia*np.cos(phi), (scale, 1))
  y3=np.tile(h_dia*np.sin(phi), (scale, 1))
  z3=np.transpose(np.tile(np.linspace(0, hei1, scale), (scale, 1)))
  # top part
  x4=np.zeros([scale, scale]); y4=np.zeros([scale, scale])
  dia=np.linspace(h_dia, 0, scale)
  for ii in range(scale):
    x4[ii]=dia[ii]*np.cos(phi)
    y4[ii]=dia[ii]*np.sin(phi)
  z4=np.transpose(np.tile(np.linspace(hei1, hei1+hei2, scale), (scale, 1)))
  # fuel part
  x5=np.tile(f_dia*np.cos(phi), (scale, 1))
  y5=np.tile(f_dia*np.sin(phi), (scale, 1))
  z5=np.transpose(np.tile(np.linspace(0.5, hei1-0.5, scale), (scale, 1)))
  # vector part
  # check
  color_vec=np.array([[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1],
                      [1, 0, 0, 1], [1, 0, 0, 1], [0, 1, 0, 1], [0, 1, 0, 1],
                      [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]])
  
  step_count=0
  
  while(1):
    step_count+=1
    try:
      shr_val=shr_memory.read()
    except:
      break
    while(shr_val[0]==119):
      try:
        shr_val=shr_memory.read()
      except:
        break
    
    loc, fuel=encode_data(shr_val)
    shr_val=shr_memory.write('w')
    if fuel>-50:
      z5=np.transpose(np.tile(np.linspace(0.5, (hei1-1)*(fuel/100)+0.5, scale), (scale, 1)))
    
    azm+=1
    if azm>180:
      azm-=360
    
    fig=plt.figure(constrained_layout=True, figsize=(10, 6))
    gs=GridSpec(6, 8, figure=fig)
    ax1=fig.add_subplot(gs[:3, 5:-1], projection="3d")
    ax2=fig.add_subplot(gs[5:, :5])
    ax3=fig.add_subplot(gs[:5, :5], projection="3d")
    ax4=fig.add_subplot(gs[3:-1, 5:-1], projection="3d")
    
    #ax1.set_title("Missile Defense System") 
    
    # ax1 reach_map
    ax1.set_box_aspect([1, 1, 1])
    ax1.set_xticks([-detection, -max_reach, 0, max_reach, detection])
    ax1.set_yticks([-detection, -max_reach, 0, max_reach, detection])
    ax1.set_zticks([0, max_reach, detection])
    ax1.plot(np.linspace(-detection, detection, scale), np.zeros(scale), np.zeros(scale), color="black")
    ax1.plot(np.zeros(scale), np.linspace(-detection, detection, scale), np.zeros(scale), color="black")
    ax1.plot_surface(x1, y1, z1, color='y', alpha=0.1)
    ax1.plot_surface(x2, y2, z2, color='g', alpha=0.2)
    ax1.view_init(elev=8, azim=azm)
    ax1.scatter(loc[0][0], loc[0][1], loc[0][2], marker='o', color="black", s=10)
    ax1.scatter(loc[1][0], loc[1][1], loc[1][2], marker='x', color="red", s=10)
    
    # ax2 text box
    ax2.axis([0, 5, 0, 5])
    ax2.axes.xaxis.set_visible(False)
    ax2.axes.yaxis.set_visible(False)
    ax2.text(0.1, 3.5, "Location")
    ax2.text(0.1, 2.5, "Veclocity")
    ax2.text(0.1, 1.5, "Acceleration")
    ax2.text(0.1, 0.5, "Fuel Remain")
    ax2.text(1, 3.5, ": %-7d:%-7d:%-7d"%(loc[0][0], loc[0][1], loc[0][2]))
    ax2.text(1, 2.5, ": %-7d:%-7d:%-7d"%(loc[2][0], loc[2][1], loc[2][2]))
    ax2.text(1, 1.5, ": %-7d:%-7d:%-7d"%(loc[3][0], loc[3][1], loc[3][2]))
    ax2.text(1, 0.5, ": %d %%"%fuel)
    
    ax2.text(3, 3.5, r"[%-7d $m$]"%np.linalg.norm(loc[0]))
    ax2.text(3, 2.5, r"[%-7d $m/s$]"%np.linalg.norm(loc[2]))
    ax2.text(3, 1.5, r"[%-7d $m/s^{2}$]"%np.linalg.norm(loc[3]))
    ax2.text(3, 0.5, r"[%-7.1f $s$]"%(step_count*T_SEC))
    
    # ax3 total_map
    ax3.set_box_aspect([1, 1, 1])
    ax3.set_xticks([-max_reach, 0, max_reach])
    ax3.set_yticks([-max_reach, 0, max_reach])
    ax3.set_zticks([0, max_reach])
    ax3.plot(np.linspace(-max_reach, max_reach, scale), np.zeros(scale), np.zeros(scale), color="black")
    ax3.plot(np.zeros(scale), np.linspace(-max_reach, max_reach, scale), np.zeros(scale), color="black")
    ax3.plot_surface(x2, y2, z2, color='g', alpha=0.2)
    ax3.view_init(elev=5, azim=azm)
    ax3.scatter(loc[0][0], loc[0][1], loc[0][2], marker='o', color="black", s=10)
    ax3.scatter(loc[1][0], loc[1][1], loc[1][2], marker='x', color="red", s=10)
    
    # ax4 missile vector
    
    # angle calculator
    x_vec=np.copy(loc[4, :]); y_vec=np.copy(loc[4, :])
    x_vec[1]=0; x_vec[2]=0
    x_vec=loc[4, :]-x_vec
    y_vec[1]=0; y_vec[2]=np.sqrt(loc[4, 1]**2+loc[4, 2]**2)
    theta_x=vec_angle(x_vec, np.array([0, 0, 1]))
    theta_y=vec_angle(y_vec, np.array([0, 0, 1]))
    velo=yx_rotation(theta_x, theta_y, loc[2, :])
    acc=yx_rotation(theta_x, theta_y, loc[3, :])
    
    soa=np.array([np.hstack([hei_mid_vec, velo/5e2]), np.hstack([hei_mid_vec, acc/1e2]), np.hstack([hei1_vec, 0, 0, 1]), np.hstack([0, 0, 0.5, np.cos(loc[5, 1])*np.sin(loc[5, 2]), np.sin(loc[5, 1])*np.sin(loc[5, 2]), np.cos(loc[5, 1])])])
    X, Y, Z, U, V, W=zip(*soa)
    ax4.set_box_aspect([1, 1, 4])
    ax4.set_xticks([-1, 1])
    ax4.set_yticks([-1, 1])
    ax4.set_zticks([-1, 11])
    ax4.plot_surface(x3, y3, z3, color="orange", alpha=0.3)
    ax4.plot_surface(x4, y4, z4, color="orange", alpha=0.3)
    ax4.plot_surface(x5, y5, z5, color="red", alpha=0.5)
    ax4.view_init(elev=15, azim=45)
    ax4.quiver(X, Y, Z, U, V, W, color=color_vec)
    
    plt.show(block=False)
    plt.pause(tt)
    plt.close()
