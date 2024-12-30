import math
import numpy as np
from tf.transformations import (quaternion_from_matrix, euler_from_matrix, 
                                quaternion_from_euler, euler_from_quaternion)

class  MATHTOOLS:
    
    # 从xyzrpy创建旋转矩阵
    def xyzrpy2Mat(self,x, y, z, roll, pitch, yaw):
        transformation_matrix = np.eye(4)
        A = np.cos(yaw)
        B = np.sin(yaw)
        C = np.cos(pitch)
        D = np.sin(pitch)
        E = np.cos(roll)
        F = np.sin(roll)
        DE = D * E
        DF = D * F
        transformation_matrix[0, 0] = A * C
        transformation_matrix[0, 1] = A * DF - B * E
        transformation_matrix[0, 2] = B * F + A * DE
        transformation_matrix[0, 3] = x
        transformation_matrix[1, 0] = B * C
        transformation_matrix[1, 1] = A * E + B * DF
        transformation_matrix[1, 2] = B * DE - A * F
        transformation_matrix[1, 3] = y
        transformation_matrix[2, 0] = -D
        transformation_matrix[2, 1] = C * F
        transformation_matrix[2, 2] = C * E
        transformation_matrix[2, 3] = z
        transformation_matrix[3, 0] = 0
        transformation_matrix[3, 1] = 0
        transformation_matrix[3, 2] = 0
        transformation_matrix[3, 3] = 1
        return transformation_matrix
    
    # 将矩阵转换为位姿
    def matrix2Pose(self,transform):
        # 提取平移部分 x, y, z
        x = transform[0, 3]
        y = transform[1, 3]
        z = transform[2, 3]
        
        # 提取旋转矩阵
        rotation_matrix = transform[:3, :3]
        
        # 将旋转矩阵转换为欧拉角 (roll, pitch, yaw)
        roll, pitch, yaw = euler_from_matrix(rotation_matrix)
        
        return x, y, z, roll, pitch, yaw
    
    def mat2xyzrpy(self,matrix):
        x = matrix[0, 3]
        y = matrix[1, 3]
        z = matrix[2, 3]
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        pitch = math.asin(-matrix[2, 0])
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
        return [x, y, z, roll, pitch, yaw]
    
    def xyzQuaternion2matrix(x, y, z, qx, qy, qz, qw):
        # 四元数到旋转矩阵：根据标准公式，将四元数分量转换为一个 3x3 的旋转矩阵。
        R = np.array([
            [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx ** 2 + qy ** 2)]
        ])
        
        # 构造齐次变换矩阵：在右下角补上 1，在第四列填入位置向量 [x, y, z]。
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        
        return T