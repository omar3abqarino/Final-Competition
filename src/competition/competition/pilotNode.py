import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import sys, tty, termios, select
import math, time
from rclpy.executors import ExternalShutdownException
from box_detection import detect_boxes



LINEAR_VEL = 5.0
ANGULAR_VEL = 1.0

# Vision-based box search/approach tuning
SEARCH_ANGULAR_VEL = 1.0
CENTER_TOLERANCE = 0.08
KP_ANGULAR = 1.2
KP_LINEAR = 3.0
STOP_HEIGHT_RATIO = 0.55

# Number of scrolls to be detected
REQUIRED_SCROLLS = 2

# Define time constants for movement since there is no odometry
THETA = 90.0
ROTATE_THETA_TIME = math.radians(THETA) / ANGULAR_VEL      # v = d / t
PAUSE_TIME = 1.5

SEARCH_MOVE_TIME = 1.0
SEARCH_LINEAR_VEL = 1.5

SAFE_DISTANCE_CM = 10


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

        # Camera subscription for box detection
        self.bridge = CvBridge()
        self.imgSub = self.create_subscription(Image, '/mono/image', self.image_callback, 10)
        self.current_box = None
        self.last_frame = None

        self.active_strategy = 2

        # Define the phase that the robot is in to help in autonomous movement
        self.strategy1_phase = "ROTATE"
        self.strategy1_timer = time.monotonic()  # Start stopwatch to estimate time

        self.strategy2_phase = "SEARCHING"

        self.obstacle_close = False

        # Subscribe to the scroll detection confirmation
        self.confirmed_count = 0
        self.confirmed_sub = self.create_subscription(Int32, '/scroll_detection_confirm', self.confirmed_callback, 10)

        # Repeat the loop every 0.05 seconds
        self.timer = self.create_timer(0.05, self.control_loop)  

        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('WASD to drive, (*) to switch to manual, (n) to switch strategy, Q to quit.')
        

    # read keys from keyboard
    def get_keys(self):
        tty.setraw(sys.stdin.fileno())
        keys = []
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            keys.append(sys.stdin.read(1))
            while True:
                rlist, _, _ = select.select([sys.stdin], [], [], 0)
                if not rlist:
                    break
                keys.append(sys.stdin.read(1))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return keys

    def control_loop(self):
        keys = self.get_keys()

        linTarget = 0.0
        linTarget1 = 0.0
        angTarget = 0.0
        
        # based on key do sth

        if '*' in keys:
            self.get_logger().info("Control Switched!!")
            self.ismanual.data = not self.ismanual.data
            self.is_manual_pub.publish(self.ismanual)
        if 'q' in keys:
            self.cmdPub.publish(Twist())
            rclpy.shutdown()
            return
        if 'n' in keys:
            self.active_strategy = 2 if self.active_strategy == 1 else 1
            if self.active_strategy == 1:
                self.strategy1_phase = "ROTATE"
                self.strategy1_timer = time.monotonic()
            else:
                self.strategy2_phase = "SEARCHING"
            self.get_logger().info(f"Switched autonomous strategy -> strategy_{self.active_strategy}")

        if self.ismanual.data:
            for key in keys:
                if key == 'w':
                    linTarget += LINEAR_VEL
                    self.get_logger().info("w") 
                elif key == 's':
                    linTarget -= LINEAR_VEL
                    self.get_logger().info("s")
                elif key == 'a':
                    angTarget += ANGULAR_VEL
                    self.get_logger().info("a")
                elif key == 'd':
                    angTarget -= ANGULAR_VEL
                    self.get_logger().info("d")
                elif key == 'k':
                    linTarget1 -= LINEAR_VEL
                    self.get_logger().info("s")
                elif key == 'j':
                    linTarget1 += LINEAR_VEL
                    self.get_logger().info("s")

            
        else:
            #automatic logic
            if self.active_strategy == 1:
                linTarget, linTarget1, angTarget = self.strategy_1()
            else:
                linTarget, linTarget1, angTarget = self.strategy_2()

        linTarget, linTarget1, angTarget = self.apply_ultrasonic_safety(linTarget, linTarget1, angTarget)

        # twist msg
        twist = Twist()
        twist.linear.x = linTarget
        twist.angular.z = angTarget
        twist.linear.y = linTarget1
        self.cmdPub.publish(twist)


    def confirmed_callback(self, msg):
        if msg.data != self.confirmed_count:
            self.confirmed_count = msg.data
            self.get_logger().info(f"Scroll Confirmed: {self.confirmed_count} out of {REQUIRED_SCROLLS}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'conversion failed: {e}')
            return

        boxes = detect_boxes(frame)
        self.frame_width = frame.shape[1]
        self.frame_height = frame.shape[0]
        self.last_frame = frame

        if boxes:
            self.current_box = max(boxes, key=lambda b: b[2] * b[3])
        else:
            self.current_box = None

    # First Strategy for autonomous
    def strategy_1(self):

        # If both scrolls are detected --> stop
        if self.confirmed_count >= REQUIRED_SCROLLS:
            return (0.0, 0.0, 0.0)

        # Start a timer
        now = time.monotonic()
        # Calculate time passed
        time_passed = now - self.strategy1_timer

        linTarget, linTarget1, angTarget = 0.0, 0.0, 0.0

        # Check the search phase
        if self.strategy1_phase == 'ROTATE':
            angTarget = ANGULAR_VEL
            # If time for rotation ends, start time for pause and search
            if time_passed >= ROTATE_THETA_TIME:
                self.strategy1_phase = 'PAUSE'
                self.strategy1_timer = now

        elif self.strategy1_phase == 'PAUSE':
            # Don't change the values of the velocities
            ...
            if time_passed >= PAUSE_TIME:
                self.strategy1_phase = 'STRAFE'
                self.strategy1_timer = now

        elif self.strategy1_phase == 'STRAFE':
            linTarget1 = SEARCH_LINEAR_VEL
            if time_passed >= SEARCH_MOVE_TIME:
                self.strategy1_phase = 'ROTATE_BACK'
                self.strategy1_timer = now

        elif self.strategy1_phase == 'ROTATE_BACK':
            linTarget1 = -ANGULAR_VEL
            if time_passed >= ROTATE_THETA_TIME:
                self.strategy1_phase = 'ROTATE'
                self.strategy1_timer = now

        # Return the values of the velocities
        return linTarget, linTarget1, angTarget

    def strategy_2(self):
        # If both scrolls are already confirmed, stop moving entirely
        if self.confirmed_count >= REQUIRED_SCROLLS:
            return (0.0, 0.0, 0.0)

        linTarget, linTarget1, angTarget = 0.0, 0.0, 0.0

        if not hasattr(self, 'frame_width') or self.current_box is None:
            self.strategy2_phase = 'SEARCHING'
            angTarget = SEARCH_ANGULAR_VEL
            return linTarget, linTarget1, angTarget

        x, y, w, h, contour = self.current_box
        bbox_center_x = x + w / 2.0
        error_x = (bbox_center_x - self.frame_width / 2.0) / (self.frame_width / 2.0)
        bbox_height_ratio = h / float(self.frame_height)

        if abs(error_x) > CENTER_TOLERANCE:
            self.strategy2_phase = 'CENTERING'
            angTarget = -KP_ANGULAR * error_x
            return linTarget, linTarget1, angTarget

        if bbox_height_ratio < STOP_HEIGHT_RATIO:
            self.strategy2_phase = 'APPROACHING'
            linTarget = KP_LINEAR * (STOP_HEIGHT_RATIO - bbox_height_ratio)
            angTarget = -KP_ANGULAR * error_x
            return linTarget, linTarget1, angTarget

        self.strategy2_phase = 'AT_BOX'
        return linTarget, linTarget1, angTarget

    def apply_ultrasonic_safety(self, linTarget, linTarget1, angTarget):
        if self.obstacle_close and linTarget > 0.0:
            linTarget = 0.0
        return linTarget, linTarget1, angTarget

    def ultrasonic_callback(self, msg):
        
        self.get_logger().info(str(msg.data))
        distance  = msg.data

        if distance <= SAFE_DISTANCE_CM:
            self.get_logger().info(f"Bad Distance = {distance}")
            self.obstacle_close = True
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.ultasonic_safety_pub.publish(msg)
        else:
            self.get_logger().info(f"Good Distance = {distance}")
            self.obstacle_close = False


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