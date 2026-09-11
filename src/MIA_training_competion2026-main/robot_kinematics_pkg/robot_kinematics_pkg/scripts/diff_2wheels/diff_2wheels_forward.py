#!/usr/bin/env python3
from numpy import pi

import rclpy 
from robot_kinematics_pkg.kinematics2.diff_kin_2wheels import DiffKin2wheels
from robot_kinematics_pkg.scripts.Ros_node import ForwardNode

def main(args=None):
    rclpy.init(args = args)

    # 79 artylon piece * 2.6cm = 2.05m (circumference) 
    R = 1.95/(2*pi) # optimo corresponding wheel radius
    gearRatio =4.625 # optimo corresponding gear to wheel ratio
    
    kin_model = DiffKin2wheels(d=0.7, r=R/gearRatio)
    forward = ForwardNode(kin_model)
    
    rclpy.spin(forward)
    
    forward.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()