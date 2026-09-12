import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys, tty, termios, select
import math, time
from rclpy.executors import ExternalShutdownException



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

        self.classificationPub = self.create_publisher(String, '/box_classification', 10)
        #test data
        self.template_bank = {
            'real': np.array([0.16, 0.0002, 1e-6, 1e-7, -1e-13, -1e-9, 1e-13]),
            'fake': np.array([0.19, 0.0009, 5e-6, 2e-7, 3e-13, 2e-9, -2e-13]),
        }
        self.classification_threshold = 0.85

        # Define the phase that the robot is in to help in autonomous movement
        self.search_phase = "SEARCHING"

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
            self.get_logger().info(f"Scroll Confirmed: {self.confirmed_count} out of {REQUIRED_SCROLLS}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'conversion failed: {e}')
            return

        boxes = self.detect_boxes(frame)
        self.frame_width = frame.shape[1]
        self.frame_height = frame.shape[0]
        self.last_frame = frame

        if boxes:
            self.current_box = max(boxes, key=lambda b: b[2] * b[3])
        else:
            self.current_box = None

    def detect_boxes(self, frame):
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        kernel = np.ones((3, 3), np.uint8)
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 500:
                continue

            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)

            if 4 <= len(approx) <= 6:
                x, y, w, h = cv2.boundingRect(approx)
            else:
                # fallback: strict polygon fit failed (noise/skew/lighting), use
                # minAreaRect on the raw contour instead of dropping it entirely
                rect = cv2.minAreaRect(c)
                (_, _), (rw, rh), _ = rect
                if rw <= 0 or rh <= 0:
                    continue
                x, y, w, h = cv2.boundingRect(c)

            aspect = w / float(h)
            if 0.6 < aspect < 1.8:
                boxes.append((x, y, w, h, c))
        return boxes

    # First Strategy for autonomous
    def auto_strategy_1(self):

        # If both scrolls are detected --> stop
        if self.confirmed_count >= REQUIRED_SCROLLS:
            return (0.0, 0.0, 0.0)

        linTarget, linTarget1, angTarget = 0.0, 0.0, 0.0

        if not hasattr(self, 'frame_width') or self.current_box is None:
            self.search_phase = 'SEARCHING'
            angTarget = SEARCH_ANGULAR_VEL
            return linTarget, linTarget1, angTarget

        x, y, w, h, contour = self.current_box
        bbox_center_x = x + w / 2.0
        error_x = (bbox_center_x - self.frame_width / 2.0) / (self.frame_width / 2.0)
        bbox_height_ratio = h / float(self.frame_height)

        if abs(error_x) > CENTER_TOLERANCE:
            self.search_phase = 'CENTERING'
            angTarget = -KP_ANGULAR * error_x
            return linTarget, linTarget1, angTarget

        if bbox_height_ratio < STOP_HEIGHT_RATIO:
            self.search_phase = 'APPROACHING'
            linTarget = KP_LINEAR * (STOP_HEIGHT_RATIO - bbox_height_ratio)
            angTarget = -KP_ANGULAR * error_x
            return linTarget, linTarget1, angTarget

        # Return the values of the velocities
        if self.search_phase != 'AT_BOX':
            self.search_phase = 'AT_BOX'
            self.classify_box(contour)
        return linTarget, linTarget1, angTarget

    def classify_box(self, contour):
        moments = cv2.moments(contour)
        hu = cv2.HuMoments(moments).flatten()
        hu_log = -np.sign(hu) * np.log10(np.abs(hu) + 1e-10)

        best_label, best_score = None, -1.0
        for label, ref in self.template_bank.items():
            ref_log = -np.sign(ref) * np.log10(np.abs(ref) + 1e-10)
            score = np.dot(hu_log, ref_log) / (
                np.linalg.norm(hu_log) * np.linalg.norm(ref_log) + 1e-8
            )
            if score > best_score:
                best_label, best_score = label, score

        if best_score < self.classification_threshold:
            result = 'unknown'
        else:
            result = best_label

        self.get_logger().info(f'Box classified as: {result} (score={best_score:.3f})')
        self.classificationPub.publish(String(data=result))

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