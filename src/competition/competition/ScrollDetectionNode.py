import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32
from cv_bridge import CvBridge
import cv2
from rclpy.executors import ExternalShutdownException
from box_detection import detect_boxes


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
        self.confirmed_count = 0

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
        detection = detect_boxes(frame)
        self.get_logger().info("after")

        if len(detection) >= self.numOfdetections:
            self.hits += 1
        else:
            self.hits = 0

        if self.hits >= self.framesNeeded and self.confirmed_count < self.numOfdetections:
            self.confirmed_count += 1
            self.hits = 0
            self.confirm_pub.publish(Int32(data=self.confirmed_count))
            self.get_logger().info(f'Scroll confirmed: {self.confirmed_count}/{self.numOfdetections}')

            if self.confirmed_count >= self.numOfdetections:
                self.done = True
                self.statusPub.publish(Bool(data=True))
                self.get_logger().info('detection done')

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