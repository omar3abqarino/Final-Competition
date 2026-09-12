#!/usr/bin/env python3
import rclpy
from robot_kinematics_pkg.kinematics1.omni4_kin import Omni4Kinematics
from robot_kinematics_pkg.scripts.Ros_node import InversNode

def main(args=None):
    rclpy.init(args=args)
    kin_model = Omni4Kinematics(L=0.23, W=0.15, R=0.044)
    invers = InversNode(kin_model)
    rclpy.spin(invers)
    invers.destroy_node()
    rclpy.shutdown()    
if __name__ == '__main__':
    main()