#!/usr/bin/env python3
import rclpy 
from robot_kinematics_pkg.kinematics2.diff_kin import DiffDriveKinematics
from robot_kinematics_pkg.scripts.Ros_node import InversNode
from numpy import pi

def main(args=None):
    rclpy.init(args = args)

    kin_model = DiffDriveKinematics(L=0.6 ,W=0.63, R = 0.06)

    invers = InversNode(kin_model)
    rclpy.spin(invers)
    invers.destroy_node()
    rclpy.shutdown()    
if __name__ == '__main__':
    main()