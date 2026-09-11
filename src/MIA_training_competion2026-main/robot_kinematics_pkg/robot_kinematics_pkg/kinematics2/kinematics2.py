#!/usr/bin/env python3

#  Base Class 
class Kinematics:
    def __init__(self, L, W, R):
        self.L = L  # length
        self.W = W  # width
        self.R = R  # wheel radius

    def forward(self, w):
        raise NotImplementedError
    

    def inverse(self, vx, vy, wz):  
        raise NotImplementedError

