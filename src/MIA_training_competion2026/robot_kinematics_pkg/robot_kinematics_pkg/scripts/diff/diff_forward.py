#!/usr/bin/env python3
from numpy import pi

import rclpy 
from robot_kinematics_pkg.kinematics2.diff_kin import DiffDriveKinematics
from robot_kinematics_pkg.scripts.Ros_node import ForwardNode

def main(args=None):
    rclpy.init(args = args)
    R = 1.975/(2*pi)
    gearRatio = 4.625
    kin_model = DiffDriveKinematics(L= .4 ,W=0.15, R = R/gearRatio)
    forward = ForwardNode(kin_model)
    rclpy.spin(forward)
    forward.destroy_node()
    rclpy.shutdown()    
if __name__ == '__main__':
    main()