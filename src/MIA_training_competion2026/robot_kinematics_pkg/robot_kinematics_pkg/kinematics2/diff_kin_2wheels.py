#!/usr/bin/env python3

import numpy as np
# from robot_kinematics_pkg.kinematics2.kinematics2 import Kinematics
from numpy import pi

#  Differential Drive (2 wheels)
class DiffKin2wheels():
    def __init__(self, d, r):   
        # r: wheel radius

        # Forward kinematics: [w_left, w_right] (RPM) -> [vx, vy, wz] (m/s, rad/s)
        self.M_forward = np.array([
            [r/2 , r/2],
            [ 0  ,  0 ],
            [-r/d, r/d]
            ])  / (60/(2*pi)) # change rpm to rad/s

        self.M_inverse = (60/(2*pi)) * np.array([
            [1/r, 0, -d/(2*r)],
            [1/r, 0,  d/(2*r)]
        ])  # change radian per second to rpm


    def forward(self, w):
        w = w[:2] # take only first 2 values of encoder speed
        w = np.array(w).reshape((2, 1))
        v = self.M_forward @ w
        return float(v[0]), float(v[1]), float(v[2])

    def inverse(self, vx, vy, wz):
        v = np.array([vx, vy, wz]).reshape((3, 1))
        w2 = self.M_inverse @ v

        w2 = w2.flatten().tolist()
        
        return w2 + w2  # [left, right, left, right]