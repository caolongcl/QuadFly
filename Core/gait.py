from math import radians
import numpy as np
import time
from . import kinematics as K
from . import robotics as R

#function neccesary to build a parametrized bezier curve 
def f(n,i):
    '''calculates binomial factor (n k)'''
    return np.math.factorial(n)/(np.math.factorial(i)*np.math.factorial(n-i))

def b(t, n, i, point):
    '''n points bezier curve'''
    return point*f(n,i)*np.power(t,i)*np.power(1-t,n-i)

class Gait:
    def __init__(self, gait_type=0):
        self.virtual_displacement = 0.0001
        self.uniform_step = 0.05
        self.bezier_curve_nb = 10
        
        self.stance_period = 0.25
        # 0 - trot
        self.gait_type = gait_type
        self.phase_offsets = []
        self.phase_offsets.append([0., 0.5, 0., 0.5])

        # 站立步态占空比
        self.stance_props = []
        self.stance_props.append(0.5)

        self.cur_phase_offset = self.phase_offsets[self.gait_type]
        self.cur_stance_prop = self.stance_props[self.gait_type]

        self.swing_period = self.stance_period / self.cur_stance_prop * (1. - self.cur_stance_prop)
        self.period = self.stance_period + self.swing_period

        self.start_time = 0.
    
    def edge(self, a):
        if np.abs(a) < 1e-6:
            return 0.0
        else:
            return a

    def stance_phase(self, phase, dir, v, vz):
        '''
        Args:
            phase： 相位，[0,1]
            dir：方向，与 x 轴正方向夹角
        Return:
            x,y,z 当前相位对应曲线的坐标
        '''
        c = self.edge(np.cos(dir))
        s = self.edge(np.sin(dir))
        
        phase_warp = 1 - 2*phase # 1 ~ 0 ~ -1

        p_stance = -self.uniform_step * phase
        
        x =  c * p_stance
        y = -s * p_stance
        z = -self.virtual_displacement*np.cos(phase_warp * np.pi * 0.5)
        
        if v != 0.:
            z = z * v
        else:
            z = z * vz
        return [x*v,y*v,z,0]

    def swing_phase(self, phase, dir, v, vz):
        c = self.edge(np.cos(dir))
        s = self.edge(np.sin(dir))

        X = np.array([-0.05, -0.06, -0.07, -0.07, 0.  ,  0.  ,  0.07,  0.07,  0.06,  0.05])*c
        Y = np.array([ 0.05,  0.06,  0.07,  0.07, 0.  , -0.  , -0.07, -0.07, -0.06, -0.05])*s
        Z = np.array([ 0.  ,  0.  ,  0.05,  0.05, 0.05,  0.06,  0.06,  0.06,  0.  ,  0.  ])

        x = 0.
        y = 0.
        z = 0.
        n = self.bezier_curve_nb

        for i in range(n): #sum all terms of the curve
            x = x + b(phase,n,i,X[i]) 
            y = y + b(phase,n,i,Y[i])
            z = z + b(phase,n,i,Z[i])
        
        if v != 0.:
            z = z * v
        else:
            z = z * vz

        return [x*v,y*v,z,0]
    
    def add(self, a, b):
        return [(a[i]+b[i]) for i in range(0,len(a))]

    def gait0(self, time, v, v_dir, vz, w, w_dir, r):
        '''
        线速度 v 及线速度方向 v_dir
        角速度 w 及角速度方向 w_dir
        r 是脚距离机体中心水平投影长度
        '''
        p = 0.
        linear = 0.
        angluar = 0.
        angle = 0.
        radius = 0.
        pos = []
        for i in range(4):
            t = time[i] - int(time[i] / self.period) * self.period
            t = np.clip(t, 0, self.period)

            angle = np.math.atan2(r[i][1], r[i][0]) - 0.5*np.pi
            radius = np.sqrt(r[i][0]**2 + r[i][1]**2)
            #print(i, np.math.degrees(angle))
            if t < self.stance_period:
                p = 1. / self.stance_period * t
                linear = self.stance_phase(p, v_dir, v, vz)
                angluar = self.stance_phase(p, angle, w * radius, w * radius)
                angluar[2] = 0.
                angluar[3] = 0.
                #print('angle', angle)
                #print('angluar', angluar)
                pos.append(self.add(linear, angluar))
            else:
                p = 1. / self.swing_period * (t - self.stance_period)
                linear = self.swing_phase(p, v_dir, v, vz)
                angluar = self.swing_phase(p, angle, w * radius, w * radius)
                angluar[2] = 0.
                angluar[3] = 0.
                pos.append(self.add(linear, angluar))
        
        return np.array(pos)

    def gait(self, v, v_dir, vz, w, w_dir, r):
        if self.start_time == 0.:
            self.start_time = time.time()
        t = time.time() - self.start_time
        
        if t > self.period:
            t = t-self.period
            self.start_time = time.time() - t
        t = np.array(self.cur_phase_offset) * self.period + t
        
        return self.gait0(t, v, v_dir, vz, w, w_dir, r)

class GaitV():
    def __init__(self, gait_type=0):
        self.virtual_displacement = 0.0001
        self.bezier_curve_nb = 10
        
        self.stance_period = 0.25
        # 0 - trot
        self.gait_type = gait_type
        self.phase_offsets = []
        self.phase_offsets.append([0., 0.5, 0., 0.5])

        # 站立步态占空比
        self.stance_props = []
        self.stance_props.append(0.5)

        self.cur_phase_offset = self.phase_offsets[self.gait_type]
        self.cur_stance_prop = self.stance_props[self.gait_type]

        self.swing_period = self.stance_period / self.cur_stance_prop * (1. - self.cur_stance_prop)
        self.period = self.stance_period + self.swing_period

        self.start_time = 0.

        # 当前周期的参数
        self.w = [0.,0.,0.]
        self.v = [0.,0.,0.]
        self.r = 0.
        self.theta = [0.,0.,0.]

        # 每周期开始检查是否更新
        self.update(self.w, self.v, self.r)

    def update(self, w, v, r):
        self.w = w
        self.v = v
        self.r = r
        self.Tms = np.array([[1.,0.,0.,0.],[0.,1.,0.,-self.r],[0.,0.,1.,0.],[0.,0.,0.,1.]])

        Sx = [1,0,0, self.v[0]/self.w[0],0,0] if self.w[0] != 0 else [0,0,0, 1,0,0]
        Sy = [0,1,0, 0,self.v[0]/self.w[1],0] if self.w[1] != 0 else [0,0,0, 0,1,0]
        Sz = [0,0,1, 0,0,self.v[0]/self.w[2]] if self.w[2] != 0 else [0,0,0, 0,0,1]
        self.Slist = np.array([Sx,Sy,Sz]).T

        for i in range(3):
            self.theta[i] = self.w[i] * self.period if self.w[i] != 0 else self.v[i] * self.period
        self.theta = np.array(self.theta)

        self.Tms1 = R.FKinSpace(self.Tms, self.Slist, self.theta)
        self.Tss1 = np.linalg.inv(self.Tms) @ self.Tms1

    def stance_phase(self, phase, dir, v, vz):
        '''
        Args:
            phase： 相位，[0,1]
            dir：方向，与 x 轴正方向夹角
        Return:
            x,y,z 当前相位对应曲线的坐标
        '''
        c = self.edge(np.cos(dir))
        s = self.edge(np.sin(dir))
        
        phase_warp = 1 - 2*phase # 1 ~ 0 ~ -1

        p_stance = -self.uniform_step * phase
        
        x =  c * p_stance
        y = -s * p_stance
        z = -self.virtual_displacement*np.cos(phase_warp * np.pi * 0.5)
        
        if v != 0.:
            z = z * v
        else:
            z = z * vz
        return [x*v,y*v,z,0]

    def swing_phase(self, phase, dir, v, vz):
        c = self.edge(np.cos(dir))
        s = self.edge(np.sin(dir))

        X = np.array([-0.05, -0.06, -0.07, -0.07, 0.  ,  0.  ,  0.07,  0.07,  0.06,  0.05])*c
        Y = np.array([ 0.05,  0.06,  0.07,  0.07, 0.  , -0.  , -0.07, -0.07, -0.06, -0.05])*s
        Z = np.array([ 0.  ,  0.  ,  0.05,  0.05, 0.05,  0.06,  0.06,  0.06,  0.  ,  0.  ])

        x = 0.
        y = 0.
        z = 0.
        n = self.bezier_curve_nb

        for i in range(n): #sum all terms of the curve
            x = x + b(phase,n,i,X[i]) 
            y = y + b(phase,n,i,Y[i])
            z = z + b(phase,n,i,Z[i])
        
        if v != 0.:
            z = z * v
        else:
            z = z * vz

        return [x*v,y*v,z,0]
    
    def add(self, a, b):
        return [(a[i]+b[i]) for i in range(0,len(a))]

    def gait0(self, time, v, v_dir, vz, w, w_dir, r):
        '''
        线速度 v 及线速度方向 v_dir
        角速度 w 及角速度方向 w_dir
        r 是脚距离机体中心水平投影长度
        '''
        p = 0.
        linear = 0.
        angluar = 0.
        angle = 0.
        radius = 0.
        pos = []
        for i in range(4):
            t = time[i] - int(time[i] / self.period) * self.period
            t = np.clip(t, 0, self.period)

            angle = np.math.atan2(r[i][1], r[i][0]) - 0.5*np.pi
            radius = np.sqrt(r[i][0]**2 + r[i][1]**2)
            #print(i, np.math.degrees(angle))
            if t < self.stance_period:
                p = 1. / self.stance_period * t
                linear = self.stance_phase(p, v_dir, v, vz)
                angluar = self.stance_phase(p, angle, w * radius, w * radius)
                angluar[2] = 0.
                angluar[3] = 0.
                #print('angle', angle)
                #print('angluar', angluar)
                pos.append(self.add(linear, angluar))
            else:
                p = 1. / self.swing_period * (t - self.stance_period)
                linear = self.swing_phase(p, v_dir, v, vz)
                angluar = self.swing_phase(p, angle, w * radius, w * radius)
                angluar[2] = 0.
                angluar[3] = 0.
                pos.append(self.add(linear, angluar))
        
        return np.array(pos)

    def gait(self, v, v_dir, vz, w, w_dir, r):
        if self.start_time == 0.:
            self.start_time = time.time()
        t = time.time() - self.start_time
        
        if t > self.period:
            t = t-self.period
            self.start_time = time.time() - t
        t = np.array(self.cur_phase_offset) * self.period + t
        
        return self.gait0(t, v, v_dir, vz, w, w_dir, r)