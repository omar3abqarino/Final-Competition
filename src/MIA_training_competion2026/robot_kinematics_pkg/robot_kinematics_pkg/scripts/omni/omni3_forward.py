#!/usr/bin/env python3
import rclpy
from robot_kinematics_pkg.kinematics1.omni3_kin import Omni3Kinematics
from robot_kinematics_pkg.scripts.Ros_node import ForwardNode

def main(args = None):
    rclpy.init(args=args)
    kin_model = Omni3Kinematics(L=0.23, W=0.67, R=0.088)
    forward = ForwardNode(kin_model)
    rclpy.spin(forward)
    forward.destroy_node()
    rclpy.shutdown()    
if __name__ == '__main__':
    main()

