#!/usr/bin/env python3
import rclpy 
from robot_kinematics_pkg.kinematics2.diff_kin import DiffDriveKinematics
from robot_kinematics_pkg.scripts.Ros_node import InversNode
from numpy import pi

def main(args=None):
    rclpy.init(args = args)
    R = 1.975/(2*pi) # optimo corresponding gear radius
    gearRatio =4.625 # optimo corresponding gear to wheel ratio
    kin_model = DiffDriveKinematics(L=0.9, W=0.7, R = R/gearRatio )  #0.334 
    invers = InversNode(kin_model)
    rclpy.spin(invers)
    invers.destroy_node()
    rclpy.shutdown()    
if __name__ == '__main__':
    main()