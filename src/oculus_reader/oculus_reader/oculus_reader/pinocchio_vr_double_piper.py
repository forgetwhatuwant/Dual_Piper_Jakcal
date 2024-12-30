#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import casadi
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin
import time
import math

from scipy.signal import medfilt

import rospy
from pinocchio import casadi as cpin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import MeshcatVisualizer
from tf.transformations import quaternion_from_euler, quaternion_from_matrix

import os
import sys
import cv2
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from piper_control import PIPER

from tools import MATHTOOLS

piper_control = PIPER()

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

class Arm_IK:
    def __init__(self):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        urdf_path = '/home/ppn/teleoperation_ws/src/cobot_remote/piper_arm/urdf/piper_description-double.urdf'

        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path)

        self.mixed_jointsToLockIDs = [#"left_base_joint",
                                      "left_joint7",
                                      "left_joint8",
                                      #"right_base_joint",
                                      "right_joint7",
                                      "right_joint8"
                                      ]

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0] * self.robot.model.nq),
        )

        # 运算工具包
        self.tools = MATHTOOLS()
        
        #q = quaternion_from_euler(0, -1.57, -1.57)
        
        #可以把逆解代码里面的这一行，换成下面的，转换到末端夹具
        first_matrix = self.tools.xyzrpy2Mat(0, 0, 0, 0, -1.57, -1.57)
        second_matrix = self.tools.xyzrpy2Mat(0.13, 0.0, 0.0, 0, 0, 0)  # 末端夹具的xyzrpy
        last_matrix = np.dot(first_matrix, second_matrix)
        q = quaternion_from_matrix(last_matrix)

        self.reduced_robot.model.addFrame(
            pin.Frame('left_ee',
                      self.reduced_robot.model.getJointId('left_joint6'),
                      pin.SE3(
                          pin.Quaternion(q[3], q[0], q[1], q[2]),
                          np.array([0.0, 0.0, 0.0]),
                      ),
                      pin.FrameType.OP_FRAME)
        )
        self.reduced_robot.model.addFrame(
            pin.Frame('right_ee',
                      self.reduced_robot.model.getJointId('right_joint6'),
                      pin.SE3(
                          pin.Quaternion(q[3], q[0], q[1], q[2]),
                          np.array([0.0, 0.0, 0.0]),
                      ),
                      pin.FrameType.OP_FRAME)
        )

        self.geom_model = pin.buildGeomFromUrdf(self.robot.model, urdf_path, pin.GeometryType.COLLISION)
        for i in range(4, 9):
            for j in range(0, 3):
                self.geom_model.addCollisionPair(pin.CollisionPair(i, j))

        for i in range(9+4, 9+9):
            for j in range(9+0, 9+3):
                self.geom_model.addCollisionPair(pin.CollisionPair(i, j))
        self.geometry_data = pin.GeometryData(self.geom_model)

        for i in range(0, 9):
            for j in range(9, 18):
                self.geom_model.addCollisionPair(pin.CollisionPair(i, j))
        self.geometry_data = pin.GeometryData(self.geom_model)

        # print("self.robot.model.nq:", self.robot.model.nq)
        # for i, joint in enumerate(self.reduced_robot.model.joints):
        #     joint_name = self.reduced_robot.model.names[i]
        #     print(f"Joint {i}: {joint_name}, ID: {joint.id}")
        # for i in range(self.reduced_robot.model.nframes):
        #    frame = self.reduced_robot.model.frames[i]
        #    frame_id = self.reduced_robot.model.getFrameId(frame.name)
        #    print(f"Frame ID: {frame_id}, Name: {frame.name}")

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.history_data = np.zeros(self.reduced_robot.model.nq)

        # # Initialize the Meshcat visualizer  for visualization
        self.vis = MeshcatVisualizer(self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model)
        self.vis.initViewer(open=True)
        self.vis.loadViewerModel("pinocchio")
        self.vis.displayFrames(True, frame_ids=[113, 114], axis_length=0.15, axis_width=5)
        self.vis.display(pin.neutral(self.reduced_robot.model))

        # Enable the display of end effector target frames with short axis lengths and greater width.
        frame_viz_names = ['L_ee_target', 'R_ee_target']
        FRAME_AXIS_POSITIONS = (
            np.array([[0, 0, 0], [1, 0, 0],
                      [0, 0, 0], [0, 1, 0],
                      [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
        )
        FRAME_AXIS_COLORS = (
            np.array([[1, 0, 0], [1, 0.6, 0],
                      [0, 1, 0], [0.6, 1, 0],
                      [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
        )
        axis_length = 0.1
        axis_width = 10
        for frame_viz_name in frame_viz_names:
            self.vis.viewer[frame_viz_name].set_object(
                mg.LineSegments(
                    mg.PointsGeometry(
                        position=axis_length * FRAME_AXIS_POSITIONS,
                        color=FRAME_AXIS_COLORS,
                    ),
                    mg.LineBasicMaterial(
                        linewidth=axis_width,
                        vertexColors=True,
                    ),
                )
            )

        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # # Get the hand joint ID and define the error function
        self.L_gripper_id = self.reduced_robot.model.getFrameId("left_ee")
        self.R_gripper_id = self.reduced_robot.model.getFrameId("right_ee")
        self.error = casadi.Function(
            "error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log6(
                        self.cdata.oMf[self.L_gripper_id].inverse() * cpin.SE3(self.cTf_l)
                    ).vector,
                    cpin.log6(
                        self.cdata.oMf[self.R_gripper_id].inverse() * cpin.SE3(self.cTf_r)
                    ).vector,
                )
            ],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        print("self.reduced_robot.model.nq:", self.reduced_robot.model.nq)
        # self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)   # for smooth
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)
        self.totalcost = casadi.sumsqr(self.error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.regularization = casadi.sumsqr(self.var_q)
        # self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last) # for smooth

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        # print("self.reduced_robot.model.lowerPositionLimit:", self.reduced_robot.model.lowerPositionLimit)
        # print("self.reduced_robot.model.upperPositionLimit:", self.reduced_robot.model.upperPositionLimit)
        self.opti.minimize(20 * self.totalcost + 0.01 * self.regularization)
        # self.opti.minimize(20 * self.totalcost + 0.01 * self.regularization + 0.1 * self.smooth_cost) # for smooth

        opts = {
            'ipopt': {
                'print_level': 0,
                'max_iter': 50,
                'tol': 1e-4,
            },
            'print_time': False
        }
        self.opti.solver("ipopt", opts)
        
        self.previous_sol_q = None  # 存储上一次的解
    
    # 计算左右臂逆解之后离目标姿态的距离
    def get_dist(self, q, left_xyz, right_xyz):
        left_index = 6
        right_index = 12
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, np.concatenate([q], axis=0))
        # print("left_xyz:", left_xyz, self.reduced_robot.data.oMi[left_index].translation)
        # print("left_xyz:", right_xyz, self.reduced_robot.data.oMi[right_index].translation)

        dist_left = math.sqrt(pow((left_xyz[0] - self.reduced_robot.data.oMi[left_index].translation[0]), 2) + pow((left_xyz[1] - self.reduced_robot.data.oMi[left_index].translation[1]), 2) + pow((left_xyz[2] - self.reduced_robot.data.oMi[left_index].translation[2]), 2))
        dist_right = math.sqrt(pow((right_xyz[0] - self.reduced_robot.data.oMi[right_index].translation[0]), 2) + pow((right_xyz[1] - self.reduced_robot.data.oMi[right_index].translation[1]), 2) + pow((right_xyz[2] - self.reduced_robot.data.oMi[right_index].translation[2]), 2))
        print("left distance：  ",dist_left, "right distance：  ",dist_right)
        return dist_left, dist_right


    def ik_fun(self, left_target_pose, right_target_pose, left_gripper=0, right_gripper=0, motorstate=None, motorV=None):
        left_gripper = np.array([left_gripper/2.0, -left_gripper/2.0])
        right_gripper = np.array([right_gripper / 2.0, -right_gripper / 2.0])
        if motorstate is not None:
            self.init_data = motorstate
        self.opti.set_initial(self.var_q, self.init_data)

        self.vis.viewer['L_ee_target'].set_transform(left_target_pose)     # for visualization
        self.vis.viewer['R_ee_target'].set_transform(right_target_pose)     # for visualization

        self.opti.set_value(self.param_tf_l, left_target_pose)
        self.opti.set_value(self.param_tf_r, right_target_pose)
        # self.opti.set_value(self.var_q_last, self.init_data) # for smooth

        try:
            # sol = self.opti.solve()
            sol = self.opti.solve_limited()
            sol_q = self.opti.value(self.var_q)

            if self.init_data is not None:
                max_diff = max(abs(self.history_data - sol_q))
                # print("max_diff:", max_diff)
                self.init_data = sol_q
                if max_diff > 10.0/180.0*3.1415:
                    # print("Excessive changes in joint angle:", max_diff)
                    self.init_data = np.zeros(self.reduced_robot.model.nq)
            else:
                self.init_data = sol_q
            self.history_data = sol_q

            self.vis.display(sol_q)  # for visualization

            if motorV is not None:
                v = motorV * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            tau_ff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v,
                              np.zeros(self.reduced_robot.model.nv))

            is_collision = self.check_self_collision(sol_q, left_gripper, right_gripper)
            
            dist_left, dist_right = self.get_dist(sol_q, left_target_pose[:3, 3], right_target_pose[:3, 3])

            return sol_q, tau_ff, is_collision, dist_left, dist_right

        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")
            # sol_q = self.opti.debug.value(self.var_q)   # return original value
            return None, '', False

    def check_self_collision(self, q, left_gripper=np.array([0, 0]), right_gripper=np.array([0, 0])):
        pin.forwardKinematics(self.robot.model, self.robot.data, np.concatenate([q[0:6], left_gripper, q[6:], right_gripper], axis=0))
        pin.updateGeometryPlacements(self.robot.model, self.robot.data, self.geom_model, self.geometry_data)
        collision = pin.computeCollisions(self.geom_model, self.geometry_data, False)
        # print("collision:", collision)
        return collision

    def smooth_solution(self, sol_q):
        # 使用中值滤波器进行平滑处理
        smoothed_q = medfilt(sol_q, kernel_size=3)
        return smoothed_q
    
    def get_ik_solution(self, left_x, left_y, left_z, left_roll, left_pitch,left_yaw, left_gripper, x, y,  
                              right_x, right_y, right_z, right_roll, right_pitch, right_yaw, right_gripper, a, b,
                              ):

        q = quaternion_from_euler(left_roll, left_pitch, left_yaw)
        left_target = pin.SE3(
            pin.Quaternion(q[3], q[0], q[1], q[2]),
            np.array([left_x, left_y, left_z]),
        )
        
        q = quaternion_from_euler(right_roll, right_pitch, right_yaw)
        right_target = pin.SE3(
            pin.Quaternion(q[3], q[0], q[1], q[2]),
            np.array([right_x, right_y, right_z]),
        )
        
        # 使用上一次的解作为初始值
        sol_q, tau_ff, is_collision, dist_left, dist_right = self.ik_fun(left_target.homogeneous, right_target.homogeneous, 0, 0)
        
        # print("PPPN:  ",sol_q[0],sol_q[1],sol_q[2], sol_q[6],sol_q[7],sol_q[8])
        
        # # 平滑处理
        # sol_q = self.smooth_solution(sol_q)
        
        # 更新上一次的解
        self.previous_sol_q = sol_q
        
       
        if  is_collision == False and y  :
            piper_control.left_joint_control_piper(sol_q[0],sol_q[1],sol_q[2],sol_q[3],sol_q[4],sol_q[5],left_gripper)
        if  is_collision == False and b  :
            piper_control.right_joint_control_piper(sol_q[6],sol_q[7],sol_q[8],sol_q[9],sol_q[10],sol_q[11],right_gripper)
    

if __name__ == "__main__":
    rospy.init_node('inverse_solution_node', anonymous=True)

    rospy.spin()
    