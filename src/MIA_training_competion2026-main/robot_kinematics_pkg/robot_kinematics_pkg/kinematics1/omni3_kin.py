#!/usr/bin/env python3
from robot_kinematics_pkg.kinematics2.kinematics2 import Kinematics
import numpy as np
import math
#  3 Wheel Omni 
class Omni3Kinematics(Kinematics):
    def __init__(self, L, W, R):
        super().__init__(L, W, R)
        self.Rb = L + W
        self.theta =  2*math.pi/3 #120 degrees between wheels
        self.gamma = 0.0 # gamma is the angular offset of the first wheel from  X_axis
        #  assuming that X_axis crossing center of first wheel

    def A_matrix(self):
        A = np.array([
            [ math.sin( self.gamma), -math.cos(  self.gamma), -self.Rb ],
            [ math.sin(self.theta + self.gamma), -math.cos(self.theta + self.gamma), -self.Rb ],
            [ math.sin(2 *self.theta + self.gamma), -math.cos(2 *self.theta + self.gamma), -self.Rb ],
        ])
        return A

    def inverse(self, vx, vy, wz):
        A = self.A_matrix()
        Vb = np.array([[vx], [vy], [wz]])
        w = (1 / self.R) * A.dot(Vb)
        return w.flatten().tolist()

    def forward(self, w):
        A = self.A_matrix()
        A_pinv = np.linalg.pinv(A)
        w = np.array(w).reshape((3, 1))
        Vb = self.R * A_pinv.dot(w)
        return float(Vb[0]), float(Vb[1]), float(Vb[2])