

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_safety_node')

        self.declare_parameter('input_vel_topic', '/cmd_vel_requested')
        self.declare_parameter('out_vel_topic', '/cmd_vel')
        self.declare_parameter('ultrasonic_topic', '/ultrasonic_distance')