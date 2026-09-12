#!/usr/bin/env python3
import rclpy 
from robot_kinematics_pkg.kinematics2.diff_kin_2wheels import DiffKin2wheels
from robot_kinematics_pkg.scripts.Ros_node import InversNode
from numpy import pi

def main(args=None):
    rclpy.init(args = args)

    # 79 artylon piece * 2.7cm = 1.975m (circumference) 
    R = 2.15/(2*pi) # optimo corresponding wheel radius
    gearRatio =4.5 # optimo corresponding gear to wheel ratio

    kin_model = DiffKin2wheels(d=0.7, r=R/gearRatio)  #0.079 
    # kin_model = DiffKin2wheels(d=0.7, r=0.065) 
    invers = InversNode(kin_model)
    
    rclpy.spin(invers)

    invers.destroy_node()
    rclpy.shutdown()    


if __name__ == '__main__':
    main()