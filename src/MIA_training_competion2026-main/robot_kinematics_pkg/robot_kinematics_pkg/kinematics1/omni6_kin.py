

import numpy as np
from robot_kinematics_pkg.kinematics2.kinematics2 import Kinematics

class Omni6Kinematics(Kinematics):
    def __init__(self , L , W , R):
        super().__init__(L , W , R)

        self.theta = np.arctan(L/W)

        # matrix for  forward kinematics
        #note:this matrix is not correct but we wont need it for the inverse kinematics
        self.M_forward = (R * 2.0 * 3.1416 / 60.0 / 4.0) * np.array([
            [1,  1,  1,  1],
            [-1, 1,  1, -1],
            [-1 / (L + W), 1 / (L + W), -1 / (L + W), 1 / (L + W)]
        ])
        # matrix for inverse kinematics
        #note scalling factor is not correct
        self.M_inverse = (60/(2*np.pi*R))*np.array([
            [1, 0, -W],
            [1, 0,  W],
            [1, 0, -W],
            [1, 0,  W],
            [0, 1,  L],
            [0, 1, -L]
        ])
        
    def forward(self, w):
        w = np.array(w).reshape((4, 1))
        v = self.M_forward @ w
        return float(v[0][0]), float(v[1][0]), float(v[2][0])

    def inverse(self, vx, vy, wz):
        v = np.array([vx, vy, wz]).reshape((3, 1))
        w = self.M_inverse @ v
        return w.flatten().tolist()