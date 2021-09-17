import numpy as np
from . import robotics as R
#import robotics as R

# 正运动学

def FKTsb(alpha, beta, gamma, xb, yb, zb):
  '''
  Args:
    alpha, beta, gamma : roll, pitch, yaw
    xb, yb, zb : 机体中心相对于世界坐标系
  Return:
    T : 机体位形
  '''
  # M1 = np.identity(4)
  # Slist1 = np.array([[1, 0, 0, 0, 0, 0],
  #                   [0, 1, 0, 0, 0, 0],
  #                   [0, 0, 1, 0, 0, 0]]).T
  M = np.array([[1, 0, 0, xb],
                [0, 1, 0, yb],
                [0, 0, 1, zb],
                [0, 0, 0, 1]])
  Slist = np.array([[1, 0, 0, 0, -zb, yb],
                    [0, 1, 0, zb, 0, -xb],
                    [0, 0, 1, -yb, xb, 0]]).T
  
  thetalist = np.array([alpha, beta, gamma])

  #print('2', R.FKinSpace(M, Slist, thetalist))
  return R.FKinSpace(M, Slist, thetalist)

def FKTbl(d1, d2, d3, d4):
  '''
  Args:
    d1, d2, d3, d4 : 4条腿肩部坐标系相对于机体坐标系各轴距离
  Return:
    T : 4条腿相对位形
  '''
  Tbl1 = np.array([[1, 0, 0, d1[0]],
                   [0, 1, 0, d1[1]],
                   [0, 0, 1, d1[2]],
                   [0, 0, 0, 1]])
  Tbl2 = np.array([[1, 0, 0, d2[0]],
                   [0, 1, 0, -d2[1]],
                   [0, 0, 1, d2[2]],
                   [0, 0, 0, 1]])
  Tbl3 = np.array([[1, 0, 0, -d3[0]],
                   [0, 1, 0, -d3[1]],
                   [0, 0, 1, d3[2]],
                   [0, 0, 0, 1]])
  Tbl4 = np.array([[1, 0, 0, -d4[0]],
                   [0, 1, 0, d4[1]],
                   [0, 0, 1, d4[2]],
                   [0, 0, 0, 1]])
  return np.array([Tbl1, Tbl2, Tbl3, Tbl4])

def Tlihi(shoulder_type, l1, theta1):
  '''
  Args:
    shoulder_type : +1 正腿（左端），-1 正腿（右端）
    l1 : 髋关节到肩关节长度
    theta1 : 肩关节角度
  Return:
    T : 髋关节相对于肩部位形
  '''
  M = np.array([[1, 0, 0, 0],
                [0, 1, 0, l1 * np.sign(shoulder_type)],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])
  S = np.array([[1, 0, 0, 0, 0, 0]]).T
  theta = np.array([theta1])

  return R.FKinSpace(M, S, theta)

def Tliki(shoulder_type, l1, l2, theta1, theta2):
  '''
  Args:
    shoulder_type : +1 正腿（左端），-1 正腿（右端）
    l1, l2 : 髋关节到肩关节长度, 膝关节到肩关节长度
    theta1, theta2 : 肩关节角度, 髋关节角度
  Return:
    T : 膝关节相对于肩部位形
  '''
  M = np.array([[1, 0, 0, 0],
                 [0, 1, 0, l1 * np.sign(shoulder_type)],
                 [0, 0, 1, -l2],
                 [0, 0, 0, 1]])
  S = np.array([[1, 0, 0, 0, 0, 0],
                 [0, 1, 0, 0, 0, 0]]).T
  theta = np.array([theta1, theta2])

  return R.FKinSpace(M, S, theta)

def Tlifi(shoulder_type, l1, l2, l3, theta1, theta2, theta3):
  '''
  Args:
    shoulder_type : +1 正腿（左端），-1 正腿（右端）
    l1, l2, l3 : 髋关节到肩关节长度, 膝关节到肩关节长度, 足部到膝关节长度
    theta1, theta2, theta3 : 肩关节角度, 髋关节角度， 膝关节角度
  Return:
    T : 足部相对于肩部位形
  '''
  M = np.array([[1, 0, 0, 0],
                [0, 1, 0, l1 * np.sign(shoulder_type)],
                [0, 0, 1, -l2-l3],
                [0, 0, 0, 1]])
  S = np.array([[1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 1, 0, l2, 0, 0]]).T
  theta = np.array([theta1, theta2, theta3])

  return R.FKinSpace(M, S, theta)

# 逆运动学

def IKlifi(leg_type, shoulder_type, x, y, z, l1, l2, l3):
  '''
  Args:
    leg_type : 肘式 1 膝盖式 -1
    shoulder_type : +1 正腿（左端），-1 正腿（右端）
    x,y,z : 足部相对于肩坐标系的坐标
    l1, l2, l3 : 关节长度
  Return:
    关节角
  '''
  L = np.sqrt(y**2+z**2-l1**2)
  D = (x**2+y**2+z**2-l1**2-l2**2-l3**2)/(2*l2*l3)
  D1 = np.sqrt(1-D**2)

  theta_1 = np.math.atan2(z,y) - np.math.atan2(-L, l1 * np.sign(shoulder_type))
  theta_3 = np.math.atan2(-np.sign(leg_type)*D1, D)

  if (leg_type * x) < 0:
    theta_2 = -np.math.atan2(-l3*np.sin(theta_3), -(l2+l3*np.cos(theta_3)))+np.math.atan2(x, -L)+2*np.pi*np.sign(leg_type)
  else:
    theta_2 = -np.math.atan2(-l3*np.sin(theta_3), -(l2+l3*np.cos(theta_3)))+np.math.atan2(x, -L)

  return np.array([theta_1, theta_2, theta_3])

def test():
  # 肘式
  thetas = [np.math.radians(-10),np.math.radians(90),np.math.radians(-90)]
  foot = Tlifi(-1, 0.5, 1., 1., 
               thetas[0], thetas[1], thetas[2])
  f = np.dot(foot,np.array([[0.],[0.],[0.],[1.0]]))
  print(f)

  theta = IKlifi(1, -1,
                 f[0,0], f[1,0], f[2,0], 
                 0.5, 1., 1.)
  print(np.math.degrees(theta[0]),np.math.degrees(theta[1]),np.math.degrees(theta[2]))

  # 膝式
  thetas = [0.,np.math.radians(-90),np.math.radians(30)]
  foot = Tlifi(-1, 0.5, 1., 1., 
               thetas[0], thetas[1], thetas[2])
  f = np.dot(foot,np.array([[0.],[0.],[0.],[1.0]]))
  print(f)

  theta = IKlifi(-1, -1,
                 f[0,0], f[1,0], f[2,0], 
                 0.5, 1., 1.)
  print(np.math.degrees(theta[0]),np.math.degrees(theta[1]),np.math.degrees(theta[2]))

if __name__ == "__main__":
    test()