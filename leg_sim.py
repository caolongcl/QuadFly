#!/bin/python

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import backup.Kinematics as K
from numpy.lib.function_base import angle

# 仿真四足腿部模型运动

fig, ax = plt.subplots()
fig.set_tight_layout(True)

plt.style.use('ggplot')
#plt.grid(zorder=0)

ax.set_aspect(1)
ax.spines['top'].set_color('none')
ax.spines['right'].set_color('none')

ax.xaxis.set_ticks_position('bottom')
ax.spines['bottom'].set_position(('data', 0))
ax.yaxis.set_ticks_position('left')
ax.spines['left'].set_position(('data', 0))

ax.xaxis.set_major_locator(plt.MultipleLocator(0.1))
ax.yaxis.set_major_locator(plt.MultipleLocator(0.1))

# 大腿和小腿长度
l1 = 0.209
l2 = 0.180

plt.xlim(-(l1 + l2) * 1.2, (l1 + l2) * 1.2)
plt.ylim(-(l1 + l2) * 1.2, l1)


#绘制初始图形
angle_1, angle_2 = np.math.radians(45), np.math.radians(-90)
# 腿构成类型 1 肘式 -1 膝式
type = 1
x, z = K.forward(l1, l2, angle_1, angle_2)
angle_1, angle_2 = K.inverse(type, l1, l2, x, z)
x0, z0 = K.forward0(l1, angle_1)
x1, z1 = K.forward(l1, l2, angle_1, angle_2)

links = ax.plot([0, x0, x1], [0, z0, z1], c='b', linewidth=3, zorder=1)[0]
trj_data_x = []
trj_data_z = []
trj = ax.plot(trj_data_x, trj_data_z, c='r', linewidth=2, zorder=0)[0]

links1 = ax.plot([0, x0, x1], [0, z0, z1], c='black', linewidth=3, zorder=1)[0]

def init():  
  links.set_data([0, x0, x1], [0, z0, z1])
  trj.set_data(trj_data_x, trj_data_z)
  return links, trj, links1

def update(i):
  t = i * stepTime / 1000.

  beta = 0.5
  T = 4
  S = 0.2
  H = 0.04
  T_sw = T*(1-beta)
  T_st = T*beta

  d = t % T
  x, z = 0, 0
  if d < T_sw:
    x = (1-beta)*S*(d/T_sw-0.5/np.pi*np.sin(2*np.pi*d/T_sw))
    f_e = d/T_sw - 0.25/np.pi*np.sin(4*np.pi*d/T_sw)
    z = H *(np.sign(0.5*T_sw-d)*(2*f_e-1)+1)
  else:
    x = beta*S*((T-d)/T_st+0.5/np.pi*np.sin(2*np.pi*d/T_st))
    z = 0
  
  angle_1, angle_2 = K.inverse(type, l1, l2, x1+x, z1+z)
  x3, z3 = K.forward0(l1, angle_1)
  x4, z4 = K.forward(l1, l2, angle_1, angle_2)

  links.set_data([0, x3, x4], [0, z3, z4])

  global trj_data_x
  global trj_data_z

  if i == 0:
    trj_data_x = []
    trj_data_z = []
    
  trj_data_x.append(x4)
  trj_data_z.append(z4)
  trj.set_data(trj_data_x, trj_data_z)

  d = (t + 0.5 * T) % T
  x, z = 0, 0
  if d < T_sw:
    x = (1-beta)*S*(d/T_sw-0.5/np.pi*np.sin(2*np.pi*d/T_sw))
    f_e = d/T_sw - 0.25/np.pi*np.sin(4*np.pi*d/T_sw)
    z = H *(np.sign(0.5*T_sw-d)*(2*f_e-1)+1)
  else:
    x = beta*S*((T-d)/T_st+0.5/np.pi*np.sin(2*np.pi*d/T_st))
    z = 0
  
  angle_1, angle_2 = K.inverse(type, l1, l2, x1+x, z1+z)
  x3, z3 = K.forward0(l1, angle_1)
  x4, z4 = K.forward(l1, l2, angle_1, angle_2)

  links1.set_data([0, x3, x4], [0, z3, z4])

  return links,trj,links1

# 每步更新 stepTime 更新周期
stepTime = 1000 * 4. / 200.

anim = FuncAnimation(fig=fig,
                      func=update,
                      frames=200,
                      init_func=init,
                      interval=stepTime,
                      blit=False)

plt.show()