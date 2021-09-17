import numpy as np

# 大腿小腿所在平面运动学 x 右，z上，y里

def forward0(l, angle):
  return [-l*np.sin(angle), -l*np.cos(angle)]

def forward(l1, l2, angle_1, angle_2):
  x0, z0 = forward0(l1, angle_1)
  dx, dz = forward0(l2, angle_1+angle_2)
  return [x0+dx, z0+dz]

def inverse(type, l1, l2, x, z):
  L_square = x**2+z**2
  L = np.sqrt(L_square)

  signs = np.sign(type)
  
  d = delta(type, x, z)

  angle_2 = -signs*np.arccos((L_square-l1**2-l2**2)/(2*l1*l2))
  angle_1 = signs*np.arcsin(-signs*l2*np.sin(angle_2)/L) - np.arctan2(z,x) - np.pi/2 + d*2*np.pi

  return [angle_1, angle_2]

def delta(type, x, z):
    if np.sign(type) == -1 and x < 0 and z >= 0:
        return 1
    else:
        return 0

# 定义机器人各种常数

class QuadParam():
  def __init__(self) -> None:
      self.omega_sw = 5*np.math.pi

      # 腿长
      self.l1 = 0.209
      self.l2 = 0.180

      # 髋关节长
      self.l0 = 0.062

      # 腿类别配置
      self.types = [1, 1, 1, 1]

      # 初始角度
      angleh = 0.24 * np.math.pi
      anglek = - 0.4 * np.math.pi
      self.init_angles = [0, self.types[0] * angleh, self.types[0] * anglek, # LF
                    0, self.types[1] * angleh, self.types[1] * anglek,  # RF
                    0, self.types[2] * angleh, self.types[2] * anglek,  # RH
                    0, self.types[3] * angleh, self.types[3] * anglek]  # LH
      
      # 足部相对于髋关节初始坐标
      self.init_foot_pos = []
      for i in range(4):
        x, z = forward(self.l1, self.l2, self.init_angles[i*3+1], self.init_angles[i*3+2])
        angle_1, angle_2 = inverse(self.types[i], self.l1, self.l2, x, z)
        x1, z1 = forward(self.l1, self.l2, angle_1, angle_2)
        self.init_foot_pos.append(x1)
        self.init_foot_pos.append(0)
        self.init_foot_pos.append(z1)
      
      # 机体坐标系
      self.body_pos = np.array([0, 0, 0])

      # 各腿髋关节坐标系原点位置
      self.hip_origin=[]

      # 步态配置
      # pace:beta, (phi_LF, phi_RLF, phi_RH, phi_LH), V, H 
      self.walk_config={'name':'walk', 'beta':0.75, 'phi':[0,0.5,0.25,0.75], 'V':0.4, 'H':0.02}
      self.trot_config={'name':'trot', 'beta':0.5, 'phi':[0,0.5,0,0.5], 'V':1.2, 'H':0.08}
      self.pace_config={'name':'pace', 'beta':0.5, 'phi':[0.01,0.5,0.5,0.01], 'V':1.,  'H':0.08}
      self.gallop_config={'name':'gallop', 'beta':0.5, 'phi':[0,0.1,0.6,0.5], 'V':1.5, 'H':0.06}
      self.stand_config={'name':'stand', 'beta':0.5, 'phi':[0,0.5,0,0.5], 'V':0.0, 'H':0.04}
      #self.bound_config={'name':'bound', 'beta':0.5, 'phi':[0,0.,0.5,0.5], 'V':0.3,  'H':0.02}
      #self.pronk_config={'name':'pronk', 'beta':0.75, 'phi':[0,0,0,0], 'V':0.3, 'H':0.02}

      configs = [self.walk_config, self.trot_config, self.pace_config, self.gallop_config, self.stand_config]
      for config in configs:
          omega_st = self.fomega_st(config['beta'])
          config['Tsw'] = np.math.pi/self.omega_sw
          config['Tst'] = np.math.pi/omega_st
          config['T'] = config['Tsw'] + config['Tst']
          config['S'] = config['T'] * config['V']
          self.config_info(config)
  
  def fomega_st(self, beta):
    return ((1-beta)/beta)*self.omega_sw

  def config_info(self, config):
    print('----- ', config['name'], ' -----')
    print('beta:', config['beta'], 'V:', config['V'],'m/s', 'H:', config['H'], 'm', 'T:', config['T'], 's')
