#!/usr/bin/env python3
import rclpy 
from robot_kinematics_pkg.diff_kin import DiffDriveKinematics
from scripts.Ros_node import InversNode
from numpy import pi

def main(args=None):
    rclpy.init(args = args)

    kin_model = DiffDriveKinematics(L=0.85 ,W=0.65, R = 0.10)
    invers = InversNode(kin_model)
    rclpy.spin(invers)
    invers.destroy_node()
    rclpy.shutdown()    
if __name__ == '__main__':
    main()