

import rclpy
import math
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_safety_node')

        # Declare Parameters
        self.declare_parameter('input_vel_topic', '/cmd_vel_requested')
        self.declare_parameter('output_vel_topic', '/cmd_vel')
        self.declare_parameter('ultrasonic_topic', '/ultrasonic_distance')
        # Get Parameters
        self.input_vel_topic = self.get_parameter('input_vel_topic').value
        self.output_vel_topic = self.get_parameter('input_vel_topic').value
        self.ultrasonic_topic = self.get_parameter('input_vel_topic').value
        # Create Publishers and Subscribers
        self.out_pub = self.create_publisher(Twist, self.output_vel_topic, 10)
        self.create_subscription(Int32, self.ultrasonic_topic, self.ultrasonic_callback, 10)
        self.create_subscription(Int32, self.input_vel_topic, self.req_vel_callback, 10)

        self.distance = math.inf

        self.get_logger().info("Ultrasonic Safety node is enabled")

    # Callback method for distance
    def ultrasonic_callback(self, msg: Int32):
        distance = int(msg.data)
        if math.isfinite(distance) and distance >= 0:
            self.distance = distance

    # Callback method for requested velocity
    def req_vel_callback(self, req_vel):
        output_vel = Twist()
        output_vel.linear.x = req_vel.linear.x
        output_vel.linear.y = req_vel.linear.y
        output_vel.angular.z = req_vel.linear.z

        if self.distance <= 20 and output_vel.linear.x > 0:
            output_vel.linear.x = 0

        self.out_pub.publish(output_vel)



def main(args = None):
    rclpy.init(args=args)
    node = UltrasonicNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()