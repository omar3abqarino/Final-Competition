#!/usr/bin/env python3
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

# ROS2 Node
class ForwardNode(Node):
    def __init__(self, kin_model):
        super().__init__('wheel_to_cmd')


        self.kin = kin_model
        self.wheel_sub = self.create_subscription(Float32MultiArray, '/encoder_speed', self.wheel_callback, 10)
        self.odom = self.create_publisher(Twist, '/forward_kinematics/speed', 10)

    def wheel_callback(self, msg):#fun. for wheel callback
        w = msg.data
        vx, vy, wz = self.kin.forward(w)
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        self.odom.publish(twist)
        self.get_logger().info("forward kinematics wheels to cmd_vel started.")
      
      
# Inverse: cmd_vel → wheels
class InversNode(Node):
    def __init__(self , kin_model):
        super().__init__('cmd_to_wheel')

        self.kin = kin_model
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.wheel_pub = self.create_publisher(Float32MultiArray, '/wheel_setpoints', 10)
        self.get_logger().info("inverse kinematics cmd_vel to wheels started.")

        
    def cmd_callback(self, msg): # fun. for cmd callback

        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        w = self.kin.inverse(vx, vy, wz)
        # self.get_logger().info(f"{vx}, {vy}, {wz}")
        array_msg = Float32MultiArray()
        array_msg.data = w
        self.wheel_pub.publish(array_msg)

