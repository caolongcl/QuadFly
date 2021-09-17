import re
import numpy as np
import backup.Kinematics as K

class QuadflyCycloid():
    def __init__(self, param):
        self.param = param
    
    def angle_phi(self, t, config, i):
        beta = config['beta']
        S = config['S']
        H = config['H']
        T_sw = config['Tsw']
        T_st = config['Tst']
        T = config['T']

        d = (t + config['phi'][i]*T) % T
        x, z = 0, 0
        if d < T_sw:
          x = (1-beta)*S*(d/T_sw-0.5/np.pi*np.sin(2*np.pi*d/T_sw))
          f_e = d/T_sw - 0.25/np.pi*np.sin(4*np.pi*d/T_sw)
          z = H *(np.sign(0.5*T_sw-d)*(2*f_e-1)+1)
        else:
          x = beta*S*((T-d)/T_st+0.5/np.pi*np.sin(2*np.pi*d/T_st))
          z = 0

        return K.inverse(self.param.types[i], self.param.l1, self.param.l2, 
              self.param.init_foot_pos[i*3]+x, self.param.init_foot_pos[i*3+2]+z)

    def angle(self, t, config):
        ang = []

        for i in range(4):
          angle_1, angle_2 = self.angle_phi(t, config, i)
          ang.append(0)
          ang.append(angle_1)
          ang.append(angle_2)

        return ang
    
    def step_ctrl(self, t, config):
        return self.angle(t, config)
        