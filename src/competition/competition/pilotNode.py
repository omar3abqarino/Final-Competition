import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import sys, tty, termios, select


SPEED_OF_SOUND = 343 / 10**(-4) # cm/us

class PilotTeleopNode(Node):
    def __init__(self):
        super().__init__('pilot_teleop_node')
        # setup pid
        self.declare_parameter('linear_target', 0.2)
        self.declare_parameter('angular_target', 1.0)
        self.linear_target = self.get_parameter('linear_target').value
        self.angular_target = self.get_parameter('angular_target').value
        self.cmdPub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ultrasonic_sensor = self.create_subscription(Float32, "/ultrasonic_distance", self.ultrasonic_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)  
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('WASD to drive, X to stop, Q to quit.')
        
    # read keys from keyb
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def control_loop(self):
        key = self.get_key()

        linTarget = 0.0
        angTarget = 0.0
        
        # based on key do smt
        if key == 'w':
            linTarget = 1.0
            self.get_logger().info("w") 
        elif key == 's':
            linTarget = -1.0
            self.get_logger().info("s")
        elif key == 'a':
            angTarget = 1.0
            self.get_logger().info("a")
        elif key == 'd':
            angTarget = -1.0
            self.get_logger().info("d")
        elif key == 'q':
            self.cmdPub.publish(Twist())
            rclpy.shutdown()
            return


        # teist msg
        twist = Twist()
        twist.linear.x = linTarget
        twist.angular.z = angTarget
        self.cmdPub.publish(twist)

    def ultrasonic_callback(self, time):
        #d = vt, d in cm.
        distance  = (time/2) * SPEED_OF_SOUND

        if distance <= 10:
            self.get_logger(f"Bad Distance = {distance}")
        else:
            self.get_logger(f"Good Distance = {distance}")


def main(args=None):
    rclpy.init(args=args)
    node = PilotTeleopNode()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
