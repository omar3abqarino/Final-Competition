import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32
from cv_bridge import CvBridge
import cv2
from rclpy.executors import ExternalShutdownException
import numpy as np



class ScrollDetectionNode(Node):
    def __init__(self):
        super().__init__('scroll_detection_node')
        # number of detc needed to be done
        self.declare_parameter('numOfdetections', 2)
        self.declare_parameter('framesNeeded', 5)

        self.numOfdetections = self.get_parameter('numOfdetections').value
        self.framesNeeded = self.get_parameter('framesNeeded').value

        self.bridge = CvBridge()
        self.hits = 0
        self.done = False

        # subscribe to image
        self.imgSub = self.create_subscription(Image, '/mono/image', self.image_callback, 10)
        # send signal to other nodes that detection done
        self.statusPub = self.create_publisher(Bool, '/scroll_detection_done', 10)
        # Publish confirmation message to the autonomous movement
        self.confirm_pub = self.create_publisher(Int32, '/scroll_detection_confirm', 10)

    def image_callback(self, msg):
        if self.done:
            return
        try:
            # read frame convert using cv2 bridge mono8 = get gray
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'conversion failed : {e}')
            return

        cv2.imshow("Camera Capture", frame)
        cv2.waitKey(1)
        self.get_logger().info("before")
        detection = self.detect(frame)
        self.get_logger().info("after")

        if len(detection) >= self.numOfdetections:
            self.hits += 1
        else:
            self.hits = 0

        # if self.hits >= self.framesNeeded:
        #     self.done = True
        #     self.statusPub.publish(Bool(data=True))
        #     self.get_logger().info('detection done')

    def detect(self, frame):
        # TODO : add model detection here, placeholder = edge/contour based
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        kernel = np.ones((3, 3), np.uint8)
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 500:
                continue
            x, y, w, h = cv2.boundingRect(c)
            aspect = w / float(h)
            if 0.7 < aspect < 1.4:
                boxes.append((x, y, w, h))
        return boxes





def main(args=None):
    rclpy.init(args=args)
    node = ScrollDetectionNode()
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