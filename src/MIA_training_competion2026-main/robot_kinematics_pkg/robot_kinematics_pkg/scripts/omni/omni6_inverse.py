#!/usr/bin/env python3
import rclpy
from robot_kinematics_pkg.kinematics1.omni6_kin import Omni6Kinematics
from robot_kinematics_pkg.scripts.Ros_node import InversNode


def main(args=None):
    rclpy.init(args=args)
    kin_model = Omni6Kinematics(L=0.60, W=0.645, R=0.044)
    invers = InversNode(kin_model)
    rclpy.spin(invers)
    invers.destroy_node()
    rclpy.shutdown()    
if __name__ == '__main__':
    main()