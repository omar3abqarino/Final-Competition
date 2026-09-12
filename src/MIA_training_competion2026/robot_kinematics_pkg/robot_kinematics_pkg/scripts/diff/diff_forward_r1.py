#!/usr/bin/env python3
from numpy import pi

import rclpy 
from robot_kinematics_pkg.diff_kin import DiffDriveKinematics
from scripts.Ros_node import ForwardNode

def main(args=None):
    rclpy.init(args = args)

    kin_model = DiffDriveKinematics(L=0.85 ,W=0.65, R = 0.10)

    forward = ForwardNode(kin_model)
    rclpy.spin(forward)
    forward.destroy_node()
    rclpy.shutdown()    
if __name__ == '__main__':
    main()