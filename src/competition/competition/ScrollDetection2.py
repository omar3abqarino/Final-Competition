import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np


SPEED_OF_SOUND = 343 / 10**(-4)  # cm/us (


class ScrollDetection(Node):
    def __init__(self):
        super().__init__('scroll_detection')

        # number of detections needed to be done
        self.declare_parameter('numOfdetections', 2)
        self.declare_parameter('framesNeeded', 5)

        self.numOfdetections = self.get_parameter('numOfdetections').value
        self.framesNeeded = self.get_parameter('framesNeeded').value

        self.bridge = CvBridge()
        self.hits = 0
        self.confirmed_count = 0

        # subscribe to image
        self.imgSub = self.create_subscription(Image, '/mono/image', self.image_callback, 10)
        # send running confirmed-scroll count to the pilot node
        self.confirmPub = self.create_publisher(Int32, '/scroll_detection_confirm', 10)
        self.ultrasonic_sensor = self.create_subscription(
            Float32, '/ultrasonic_distance', self.ultrasonic_callback, 10
        )

    def image_callback(self, msg):
        if self.confirmed_count >= self.numOfdetections:
            return
        try:
            # read frame, convert using cv2 bridge, mono8 = grayscale
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'conversion failed: {e}')
            return

        boxes = self.detect(frame)

        if len(boxes) >= self.numOfdetections:
            self.hits += 1
        else:
            self.hits = 0

        if self.hits >= self.framesNeeded and self.confirmed_count < self.numOfdetections:
            self.confirmed_count += 1
            self.hits = 0
            self.confirmPub.publish(Int32(data=self.confirmed_count))
            self.get_logger().info(
                f'Scroll confirmed: {self.confirmed_count}/{self.numOfdetections}'
            )

    def detect(self, frame):
        # apply filters to get boxes
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        kernel = np.ones((3, 3), np.uint8)
        morph = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []
        for c in contours:
            a = cv2.contourArea(c)
            if a < 500:
                continue
            x, y, w, h = cv2.boundingRect(c)
            aspect = w / float(h)
            if 0.7 < aspect < 1.4:
                boxes.append((x, y, w, h))

        return boxes

    def ultrasonic_callback(self, msg):
        # d = vt, d in cm.
        distance = (msg.data / 2) * SPEED_OF_SOUND

        if distance <= 10:
            self.get_logger().warn(f'Bad Distance = {distance}')
        else:
            self.get_logger().info(f'Good Distance = {distance}')


def main(args=None):
    rclpy.init(args=args)
    node = ScrollDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()