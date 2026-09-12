#!/usr/bin/env python3
import rclpy
from robot_kinematics_pkg.kinematics2.mecanum_kin import MecanumKinematics
from robot_kinematics_pkg.scripts.Ros_node import ForwardNode

def main(args = None):
    rclpy.init(args=args)
    kin_model = MecanumKinematics(L = 0.45, W = 0.45, R = 0.07)
    forward = ForwardNode(kin_model)
    rclpy.spin(forward)
    forward.destroy_node()
    rclpy.shutdown()    
if __name__ == '__main__':
    main()