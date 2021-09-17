#!/bin/python
# -*- coding: utf-8 -*-

import pybullet
import pybullet_data

import time
import numpy as np

from pybulletdebug import PybulletDebug
from Core.quadruplet import Quadruplet
from Core.gait import Gait

# 四足仿真环境


class QuadrupedSim:
    def __init__(self) -> None:
        self.client = pybullet
        self.client_data = pybullet_data
        self.client.connect(self.client.GUI)
        self.client.setAdditionalSearchPath(self.client_data.getDataPath())

        # 设置为 0 以禁用隐式圆锥摩擦并使用金字塔近似值（默认为圆锥体）
        self.client.setPhysicsEngineParameter(enableConeFriction=1)
        self.client.setGravity(0, 0, -9.8)
        #self.client.setGravity(0, 0, 0)

        # 设置初始摄像机视角
        #self.client.resetDebugVisualizerCamera(2, 135, -15, [0, 0, 1])
        self.client.resetDebugVisualizerCamera(2.0, 0, 0, [0, 0, 0.4])

        # 设置仿真步长
        self.time_step = 1. / 240.
        self.client.setTimeStep(self.time_step)

        self.pybullet_debug = PybulletDebug()
        self.quadruplet = Quadruplet()
        self.gait = Gait()

        self.init_sfoots = []

        # 状态复位
        self.reset()

    def exit(self):
        quadrupedPos, quadrupedOrn = self.client.getBasePositionAndOrientation(
            self.quadruped)
        print(quadrupedPos, quadrupedOrn)
        self.client.disconnect()

    def reset(self):
        self.plane = self.client.loadURDF('plane.urdf')

        # 加载模型
        quadrupedStartPos = [0, 0, .4]
        # p.getQuaternionFromEuler([0,0,0])
        quadrupedStartOrientation = [0, 0.5, 0.5, 0]
        urdfFlags = self.client.URDF_USE_SELF_COLLISION
        self.quadruped = self.client.loadURDF("mini_cheetah/mini_cheetah.urdf",
                                              quadrupedStartPos,
                                              # quadrupedStartOrientation,
                                              # flags=urdfFlags,
                                              useFixedBase=False)
        # useFixedBase=True)

        # 记录joints 和 四足位置
        self.joints_var = [[], [], [], []]
        self.foot_pos = [[], [], [], []]
        self.leg_sign = [-1, -1, 1, 1]

        # 设置各部分颜色，便于区分
        self.client.changeVisualShape(
            self.quadruped, -1, rgbaColor=[1, 1, 1, 1])
        color = [[1, 0, 0, 1], [1, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 1],
                 [1, 0, 0, 1], [1, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 1],
                 [1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 1],
                 [1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 1]]

        # 打印关节信息
        print("[INFO] !!!Joints info!!!")
        for j in range(self.client.getNumJoints(self.quadruped)):
            self.client.changeVisualShape(
                self.quadruped, j, rgbaColor=color[j])
            #self.client.changeDynamics(self.quadruped, j, linearDamping=0, angularDamping=0)
            #print(self.client.getJointInfo(self.quadruped, j))

        # joint的旋转轴方向
        self.joint_directions = [[1, -1, -1],  # LF
                                 [1, -1, -1],  # RF
                                 [1, -1, -1],  # RH
                                 [1, -1, -1]]  # LH
        # 给joint编号
        self.joint_ids = [[4, 5, 6], [0, 1, 2], [8, 9, 10], [12, 13, 14]]
        self.joint_names = []
        for j in range(self.client.getNumJoints(self.quadruped)):
            self.joint_names.append(
                self.client.getJointInfo(self.quadruped, j)[1])

        pos, orn, v, v_dir, w, w_dir = self.pybullet_debug.cam_and_robotstates(
            self.quadruped)
        self.init_sfoots = self.quadruplet.FKSolve(
            orn, pos, self.quadruplet.init_thetas)
        thetas = self.quadruplet.IKSolve(orn, pos, self.init_sfoots)

        print(self.quadruplet.init_thetas)
        print(thetas)

        # 初始化位姿
        for i in range(4):
            self.set_pos(i, thetas[i][0], thetas[i][1], thetas[i][2])

    def set_pos(self, leg_id, hip_angle, upper_angle, lower_angle):
        pos = [hip_angle, upper_angle, lower_angle]
        final_pos = np.array(self.joint_directions[leg_id]) * np.array(pos)
        force = [20.] * len(self.joint_directions[leg_id])
        self.client.setJointMotorControlArray(self.quadruped, self.joint_ids[leg_id],
                                              self.client.POSITION_CONTROL,
                                              final_pos)

    def step(self):
        self.client.setRealTimeSimulation(0)

        footpos = self.init_sfoots

        while not self.needExit():
            pos, orn, v, v_dir, w, w_dir = self.pybullet_debug.cam_and_robotstates(
                self.quadruped)

            footpos = self.gait.gait(
                v, v_dir, 0, w, w_dir, footpos) + self.init_sfoots

            thetas = self.quadruplet.IKSolve(orn, pos, footpos)

            for i in range(4):
                self.set_pos(i, thetas[i][0], thetas[i][1], thetas[i][2])
                #print('f%d:%.4f %.4f %.4f' % (i,thetas[i][0],thetas[i][1],thetas[i][2]))
            self.client.stepSimulation()
            time.sleep(self.time_step)

    def needExit(self):
        qKey = ord('q')
        keys = self.client.getKeyboardEvents()
        if qKey in keys and keys[qKey] & self.client.KEY_IS_DOWN:
            return True
        else:
            return False


if __name__ == "__main__":
    dog = QuadrupedSim()
    dog.step()
    dog.exit()
