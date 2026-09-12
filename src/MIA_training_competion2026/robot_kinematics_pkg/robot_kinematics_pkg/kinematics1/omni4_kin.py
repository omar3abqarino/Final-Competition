#!/usr/bin/env python3
import math 
import rclpy
import numpy as np
from robot_kinematics_pkg.kinematics2.kinematics2 import Kinematics

#  inhert Omni 4 (X-drive)
class Omni4Kinematics(Kinematics):
    def __init__(self, L, W, R):
        super().__init__(L, W, R)
        self.summ =math.sqrt( L**2 + W**2) #distance from robot center to wheel center
        # 4 wheels  positions (x,y) around the robot center
        # wheel positions
        self.theta = math.pi/2  #theta = (2pi/n) ⇒ n= num of wheels
        # omni rollers axis is parallel to wheel tangent
        self.gamma = math.pi/4 
        #matrix A for invers kinematics
        self.A = np.array([ 
            [ math.sin(self.gamma), -math.cos( self.gamma), -self.summ], 
            [ math.sin(self.theta + self.gamma), -math.cos(self.theta + self.gamma), -self.summ],
            [ math.sin(2*self.theta + self.gamma), -math.cos(2 *self.theta + self.gamma), -self.summ], 
            [ math.sin(3*self.theta + self.gamma), -math.cos(3*self.theta + self.gamma), -self.summ], 
        ])

    def inverse(self, vx, vy, wz):
        # angular velocity = (1/R) * A * [vx, vy, ωz]transformation
    
        Vb = np.array([[vx], [vy], [wz]])
        print(Vb)
        w = (1 / self.R) * np.matmul(self.A , Vb)

        return w.flatten().tolist()

    def forward(self, w):
        # [vx, vy, ωz]transformation = R * pinv(A) * ω
        A_pinv = np.linalg.pinv(self.A) #invers matrix for A
         # inverse of A
        w = np.array(w).reshape((4, 1))
        Vb = self.R * np.matmul(A_pinv ,w)
        return float(Vb[0]), float(Vb[1]), float(Vb[2])