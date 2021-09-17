import pybullet as p
import time
import numpy as np
import sys

class PybulletDebug:
    def __init__(self):
        #Camera paramers to be able to yaw pitch and zoom the camera (Focus remains on the robot) 
        self.cyaw=90
        self.cpitch=-7
        self.cdist=0.66
        time.sleep(0.5)
        
        self.xId = p.addUserDebugParameter("x" , -0.10 , 0.10 , 0.)
        self.yId = p.addUserDebugParameter("y" , -0.10 , 0.10 , 0.)
        self.zId = p.addUserDebugParameter("z" , -0.10 , 0.10 , 0.)
        self.rollId = p.addUserDebugParameter("roll" , -45. , 45. , 0.)
        self.pitchId = p.addUserDebugParameter("pitch" , -45. , 45. , 0.)
        self.yawId = p.addUserDebugParameter("yaw" , -45. , 45. , 0.)

        self.vId = p.addUserDebugParameter("v" , 0. , 4. , 0.)
        self.v_dirId = p.addUserDebugParameter("v_dir" , -180. , 180. , 0.)
        self.wId = p.addUserDebugParameter("w" , 0. , 10. , 0.)
        self.w_dirId = p.addUserDebugParameter("w_dir" , -1. , 1. , 1.)
  
    def cam_and_robotstates(self , boxId):
                ####orientacion de la camara
        cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
        p.resetDebugVisualizerCamera( cameraDistance=self.cdist, cameraYaw=self.cyaw, cameraPitch=self.cpitch, cameraTargetPosition=cubePos)
        keys = p.getKeyboardEvents()
        #Keys to change camera
        if keys.get(100):  #D
            self.cyaw+=1
        if keys.get(97):   #A
            self.cyaw-=1
        if keys.get(99):   #C
            self.cpitch+=1
        if keys.get(102):  #F
            self.cpitch-=1
        if keys.get(122):  #Z
            self.cdist+=.01
        if keys.get(120):  #X
            self.cdist-=.01
        if keys.get(27):  #ESC
            p.disconnect()
            sys.exit()
        #read position from debug
        pos = np.array([p.readUserDebugParameter(self.xId),p.readUserDebugParameter(self.yId), p.readUserDebugParameter(self.zId)])
        orn0 = np.math.radians(p.readUserDebugParameter(self.rollId))
        orn1 = np.math.radians(p.readUserDebugParameter(self.pitchId))
        orn2 = np.math.radians(p.readUserDebugParameter(self.yawId))
        orn = np.array([orn0,orn1, orn2])

        v = p.readUserDebugParameter(self.vId)
        v_dir = np.math.radians(p.readUserDebugParameter(self.v_dirId))

        w = p.readUserDebugParameter(self.wId)
        w_dir = p.readUserDebugParameter(self.w_dirId)
        
        return pos , orn , v, v_dir,w,w_dir
