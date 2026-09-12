import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32
from geometry_msgs.msg import Twist
import sys, tty, termios, select
import math, time
from rclpy.executors import ExternalShutdownException



LINEAR_VEL = 5.0
ANGULAR_VEL = 1.0

# Define time constants for movement since there is no odometry
THETA = 90.0
ROTATE_THETA_TIME = math.radians(THETA) / ANGULAR_VEL      # v = d / t
PAUSE_TIME = 1.5

SEARCH_MOVE_TIME = 1.0
SEARCH_LINEAR_VEL = 1.5

# Number of scrolls to be detected
REQUIRED_SCROLLS = 2


class PilotTeleopNode(Node):
    def __init__(self):
        super().__init__('pilot_teleop_node')
        # Manual Messages
        self.ismanual = Bool()
        self.ismanual.data = False
        self.is_manual_pub = self.create_publisher(Bool, '/is_manual', 10)

        # Parameters for linear and angular velocities
        self.declare_parameter('linear_target', 0.2)
        self.declare_parameter('angular_target', 1.0)

        self.linear_target = self.get_parameter('linear_target').value
        self.angular_target = self.get_parameter('angular_target').value

        self.cmdPub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Ultrasonic gateway topic
        self.ultasonic_safety_pub = self.create_publisher(Twist, '/cmd_vel_requested', 10)
        self.ultrasonic_sensor = self.create_subscription(Int32, "/ultrasonic_distance", self.ultrasonic_callback, 10)
        
        # Define the phase that the robot is in to help in autonomous movement
        self.search_phase = "ROTATE"
        self.search_timer = time.monotonic()  # Start stopwatch to estimate time

        # Subscribe to the scroll detection confirmation
        self.confirmed_count = 0
        self.confirmed_sub = self.create_subscription(Int32, '/scroll_detection_confirm', self.confirmed_callback, 10)

        # Repeat the loop every 0.05 seconds
        self.timer = self.create_timer(0.05, self.control_loop)  

        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('WASD to drive, (*) to switch to manual, Q to quit.')
        

    # read keys from keyboard
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def control_loop(self):
        key = self.get_key()

        linTarget = 0.0
        linTarget1 = 0.0
        angTarget = 0.0
        
        # based on key do sth

        if key == '*':
            self.get_logger().info("Control Switched!!")
            self.ismanual.data = not self.ismanual.data
            self.is_manual_pub.publish(self.ismanual)
        elif key == 'q':
            self.cmdPub.publish(Twist())
            rclpy.shutdown()
            return

        if self.ismanual.data:
            if key == 'w':
                linTarget = LINEAR_VEL
                self.get_logger().info("w") 
            elif key == 's':
                linTarget = -LINEAR_VEL
                self.get_logger().info("s")
            elif key == 'a':
                angTarget = ANGULAR_VEL
                self.get_logger().info("a")
            elif key == 'd':
                angTarget = -ANGULAR_VEL
                self.get_logger().info("d")
            elif key == 'k':
                linTarget1 = -LINEAR_VEL
                self.get_logger().info("s")
            elif key == 'j':
                linTarget1 = LINEAR_VEL
                self.get_logger().info("s")

            
        else:
            #automatic logic
            linTarget, linTarget1, angTarget = self.auto_strategy_1()
        
            

        # twist msg
        twist = Twist()
        twist.linear.x = linTarget
        twist.angular.z = angTarget
        twist.linear.y = linTarget1
        self.cmdPub.publish(twist)


    def confirmed_callback(self, msg):
        if msg.data != self.confirmed_count:
            self.confirmed_count = msg.data
            self.get_logger.info(f"Scroll Confirmed: {self.confirmed_count} out of {REQUIRED_SCROLLS}")

    # First Strategy for autonomous
    def auto_strategy_1(self):
        
        # If both scrolls are detected --> stop
        if self.confirmed_count >= REQUIRED_SCROLLS:
            return (0.0, 0.0, 0.0)

        # Start a timer
        now = time.monotonic()
        # Calculate time passed
        time_passed = now - self.search_timer

        linTarget, linTarget1, angTarget = 0.0, 0.0, 0.0

        # Check the search phase
        if self.search_phase == 'ROTATE':
            angTarget = ANGULAR_VEL
            # If time for rotation ends, start time for pause and search
            if time_passed >= ROTATE_THETA_TIME:
                self.search_phase = 'PAUSE'
                self.search_timer = now

        elif self.search_phase == 'PAUSE':
            # Don't change the values of the velocities
            ...
            if time_passed >= PAUSE_TIME:
                self.search_phase = 'STRAFE'
                self.search_timer = now

        elif self.search_phase == 'STRAFE':
            linTarget1 = SEARCH_LINEAR_VEL
            if time_passed >= SEARCH_MOVE_TIME:
                self.search_phase = 'ROTATE_BACK'
                self.search_timer = now

        elif self.search_phase == 'ROTATE_BACK':
            linTarget1 = -ANGULAR_VEL
            if time_passed >= ROTATE_THETA_TIME:
                self.search_phase = 'ROTATE'
                self.search_timer = now

        # Return the values of the velocities
        return linTarget, linTarget1, angTarget
        
        

        

    def ultrasonic_callback(self, msg):
        
        self.get_logger().info(str(msg.data))
        distance  = msg.data

        if distance <= 10:
            self.get_logger().info(f"Bad Distance = {distance}")
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.ultasonic_safety_pub.publish(msg)
        else:
            self.get_logger().info(f"Good Distance = {distance}")


def main(args=None):
    rclpy.init(args=args)
    node = PilotTeleopNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
