#!/bin/python
# -*- coding: utf-8 -*-

import pybullet
import pybullet_data

import time
import numpy as np
import backup.Kinematics as K
import quad_ctrl as qc
import quad_ctrl_cycloid as qcc

# 四足仿真环境


class QuadrupedSim:
    def __init__(self, control, config) -> None:
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

        self.control = control
        self.config = config
        self.t = 0

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

        # lower_legs = [2, 5, 8, 11]
        # for l0 in lower_legs:
        #   for l1 in lower_legs:
        #     if (l1 > l0):
        #       enableCollision = 1
        #       print("collision for pair", l0, l1,
        #             self.client.getJointInfo(self.quadruped, l0)[12],
        #             self.client.getJointInfo(self.quadruped, l1)[12], "enabled=", enableCollision)
        #       self.client.setCollisionFilterPair(self.quadruped, self.quadruped, l0, l1,  enableCollision)

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

        # 初始化位姿
        for i in range(4):
            self.set_pos(i, self.control.param.init_angles[i*3],
                         self.control.param.init_angles[i*3+1],
                         self.control.param.init_angles[i*3+2])

    def set_pos(self, leg_id, hip_angle, upper_angle, lower_angle):
        pos = [hip_angle, upper_angle, lower_angle]
        final_pos = np.array(self.joint_directions[leg_id]) * np.array(pos)
        force = [20.] * len(self.joint_directions[leg_id])
        self.client.setJointMotorControlArray(self.quadruped, self.joint_ids[leg_id],
                                              self.client.POSITION_CONTROL,
                                              final_pos, forces=force)

    def step(self):
        init_t = 0

        # self.client.setRealTimeSimulation(1)
        while init_t <= 120 * 1:
            init_t += 1
            self.client.stepSimulation()
            time.sleep(self.time_step)

        while not self.needExit():
            self.run(self.time_step)

            self.client.stepSimulation()
            time.sleep(self.time_step)

    def step_once(self, func):
        t = 0
        func(t)

        while not self.needExit():
            self.client.stepSimulation()
            time.sleep(self.time_step)

    def needExit(self):
        qKey = ord('q')
        keys = self.client.getKeyboardEvents()
        if qKey in keys and keys[qKey] & self.client.KEY_IS_DOWN:
            return True
        else:
            return False

    def run(self, step):
        pos = self.control.step_ctrl(self.t, self.config)
        for i in range(4):
            self.set_pos(i, pos[i*3], pos[i*3+1], pos[i*3+2])

        self.t += step


if __name__ == "__main__":
    #ctrl = qc.Quadfly()
    param = K.QuadParam()
    ctrl = qcc.QuadflyCycloid(param)
    dog = QuadrupedSim(ctrl, ctrl.param.pace_config)
    dog.step()
    dog.exit()
