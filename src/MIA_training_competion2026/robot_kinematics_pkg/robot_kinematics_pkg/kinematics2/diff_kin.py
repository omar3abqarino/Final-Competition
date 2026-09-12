#!/usr/bin/env python3

import numpy as np
from robot_kinematics_pkg.kinematics2.kinematics2 import Kinematics
from numpy import pi
#  Differential Drive 
class DiffDriveKinematics(Kinematics):
    def __init__(self, L , W, R):
        super().__init__( L , W , R)


        self.M_forward = np.array([
            [R/4,  R/4  ,R/4, R/4],
            [0,    0  , 0,   0],
            [-R/(4*L), R/(4*L), -R/(4*L), R/(4*L)]
        ])/(60/(2*pi)) # change rpm to rad/s

        # change radian per second to rpm
        self.M_inverse = (60/(2*pi)) * np.array([
            [1/R, 0, -L/R],
            [1/R, 0,  L/R],
            [1/R, 0, -L/R],
            [1/R, 0,  L/R]
        ])


    def forward(self, w):
        w = np.array(w).reshape((4, 1))
        v = self.M_forward @ w  
        return float(v[0][0]), float(v[1][0]),  float(v[2][0])

    def inverse(self, vx, vy, wz):
        v = np.array([vx, vy, wz]).reshape((3, 1))
        w = self.M_inverse @ v
        return w.flatten().tolist()