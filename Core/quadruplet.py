import numpy as np
from . import kinematics as K

class Quadruplet():
  def __init__(self):
    # self.L = 
    # self.W = 
    self.d1 = np.array([0.19, 0.049, 0.0])
    self.d2 = np.array([0.19, 0.049, 0.0])
    self.d3 = np.array([0.19, 0.049, 0.0])
    self.d4 = np.array([0.19, 0.049, 0.0])
    self.l1 = 0.062
    self.l2 = 0.209
    self.l3 = 0.18
    # 关节类：1 为肘式 -1 为膝式
    self.leg_type = [1,1,1,1]
    # 腿方位：1 腿在肩的左边 -1 腿在肩的右边
    self.shoulder_type = [1,-1,-1, 1]

    self.init_thetas = np.array([[0.,np.math.radians(45),np.math.radians(-90)],
                                 [0.,np.math.radians(45),np.math.radians(-90)],
                                 [0.,np.math.radians(45),np.math.radians(-90)],
                                 [0.,np.math.radians(45),np.math.radians(-90)]])

  def FKSolve(self, orn, pos, thetas):
    Tsb = K.FKTsb(orn[0], orn[1], orn[2], pos[0], pos[1], pos[2])
    Tbl = K.FKTbl(self.d1, self.d2, self.d3, self.d4)
    
    sfoots = np.array([[0.,0.,0.,1.],[0.,0.,0.,1.],[0.,0.,0.,1.],[0.,0.,0.,1.]])
    origin = np.array([[0.],[0.],[0.],[1.0]])

    for i in range(4):
      theta = thetas[i]
      Tlf = K.Tlifi(self.shoulder_type[i], self.l1, self.l2, self.l3, theta[0], theta[1], theta[2])
      Tsf = Tsb @ Tbl[i] @ Tlf
      # print(Tsf)
      # print(origin)
      # print(np.dot(Tsf, origin))
      sfoots[i] = np.reshape(np.dot(Tsf, origin), (4,))
    
    return sfoots

  def IKSolve(self, orn, pos, sfoots):
    '''
    Args:
      orn : 机体方向
      pos : 机体与机体初始坐标系相对位置
      sfoot : 4条腿足底坐标相对机体初始坐标系的齐次坐标，列对应
    Return:
      关节角
    '''
    # Tsb = K.FKTsb(orn[0], orn[1], orn[2], pos[0], pos[1], pos[2])
    # Tbl = K.FKTbl(self.d1, self.d2, self.d3, self.d4)
    
    # thetas = np.array([[0.,0.,0.],[0.,0.,0.],[0.,0.,0.],[0.,0.,0.]])
    # for i in range(4):
    #   Tsl = Tsb @ Tbl[i]
    #   lfoot = np.linalg.inv(Tsl) @ sfoots[i]
    #   thetas[i] = K.IKlifi(self.leg_type[i], lfoot[0], lfoot[1], lfoot[2], self.l1, self.l2, self.l3)
    
    Tsb = K.FKTsb(orn[0], orn[1], orn[2], pos[0], pos[1], pos[2])
    Tbl = K.FKTbl(self.d1, self.d2, self.d3, self.d4)
    
    thetas = np.array([[0.,0.,0.],[0.,0.,0.],[0.,0.,0.],[0.,0.,0.]])
    for i in range(4):
      lfoot = np.linalg.inv(Tsb @ Tbl[i]) @ sfoots[i]
      thetas[i] = K.IKlifi(self.leg_type[i], self.shoulder_type[i], lfoot[0], lfoot[1], lfoot[2], self.l1, self.l2, self.l3)

    return thetas
  
  def FKOrnSolve(self, orn, sfoots):
    sfoots1 = np.array([[0.,0.,0.,1.],[0.,0.,0.,1.],[0.,0.,0.,1.],[0.,0.,0.,1.]])
    Tsb = K.FKTsb(0., 0., orn[2], 0., 0., 0.)

    for i in range(4):
      sfoots1[i] = np.linalg.inv(Tsb) @ sfoots[i]
      # sfoots1[i] = Tsb @ sfoots[i]
    
    return sfoots1 - sfoots