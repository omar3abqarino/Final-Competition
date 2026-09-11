import cv2
import rclpy
import numpy as np
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image, CompressedImage
from rclpy.node import Node
from srv_pkg.srv import SendImg
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from .cameraclass import qos_profile
import time

node_name_counter = 0

class cameradriver(Node):

    def __init__(self, subscriber=False, client=False, topic="camera_speaker", node_name="camera_listener", service="camera_service", compressed = False):
        
        global node_name_counter
        self.node_name = f"{node_name}_{node_name_counter}"
        node_name_counter += 1
        super().__init__(self.node_name)

        self.service = service
        self.topic = topic
        self.br = CvBridge()
        self.subscriberMode=subscriber
        self.clientMode=client      
        self.current_frame = None
        self.compressed = compressed

        self.msg = None
        self.timestamp = None
        self.spin_success = False

       #----------set mode------------#
    def set_mode(self):
        if self.subscriberMode:
            self.init_subscriber()
        if self.clientMode:
            self.init_client()


    #--------------subscriber code---------#
    def init_subscriber(self): # overwritten in kinect_v1
        if self.compressed:
            self.subscription = self.create_subscription(CompressedImage, self.topic, self.subscribe, qos_profile)
            self.get_logger().info('init compressed subscriber')
        else:
            self.subscription = self.create_subscription(Image, self.topic, self.subscribe, qos_profile)
            self.get_logger().info('init subscriber')

    def subscribe(self,msg):
        self.msg = msg
        self.timestamp = msg.header.stamp
        if self.compressed:
            np_arr = np.frombuffer(self.msg.data, np.uint8)
            self.current_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.get_logger().info('Received a frame')
        else:
            self.current_frame = self.br.imgmsg_to_cv2(self.msg)

    def display(self): # overwritten in kinect_v1
        if self.current_frame is not None:
            cv2.imshow(f'displaying {self.node_name}', self.current_frame)
            self.get_logger().info('displaying')
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                rclpy.shutdown()
        else: 
            self.get_logger().warn("No images recieved to display")

    def get_frame(self): # overwritten in kinect_v1
        return self.current_frame

    #---------------client code---------------#
    def init_client(self):
        self.client = self.create_client(SendImg, self.service)

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...') 
        self.request = SendImg.Request()
        self.request.req = True

    def get_frame_client(self):
        if self.current_frame is None:
            return        
        return self.current_frame

    def send_request(self):          
        self.future = self.client.call_async(self.request)

        rclpy.spin_until_future_complete(self, self.future)
        self.current_frame = self.br.imgmsg_to_cv2(self.future.result().img)
        
        self.get_logger().info("received message")
        return self.current_frame

    def spin(self, timeout):
        start = time.monotonic()
        rclpy.spin_once(self, timeout_sec=timeout)
        elapsed = time.monotonic() - start

        if timeout is not None and elapsed >= timeout:
            self.get_logger().warn(f"\n -> camera Subscriber spin_once timed out after {elapsed:.3f}s\n")
            self.spin_success = False
        else: 
            self.spin_success = True
        rclpy.spin_once(self, timeout_sec=timeout)

    def get_timestamp(self):
        return self.timestamp
    
    def get_internsic_values():
        pass

class monoDriver(cameradriver):
    def __init__(self,subscriber=True, client=False, topic="mono", node_name="mono_node_sub", service="mono_service", compressed = False):
        super().__init__(subscriber, client, topic, node_name, service, compressed)
        super().set_mode()

    def get_internsic_values(self):
        focal_length = 227.62354385*3
        Fx = Fy = focal_length
        Cx = 178.53
        Cy = 187.09
        return Fx , Fy, Cx , Cy

class stereoDriver(cameradriver):
    def __init__(self,subscriber=True, client=False, topic="stereo", node_name="stereo_node_sub", service="stereo_service", compressed = False):
        super().__init__(subscriber, client, topic, node_name, service, compressed)
        super().set_mode()

    def get_internsic_values(self):
        focal_length = 194
        Fx = Fy = focal_length 
        Cx = 320
        Cy = 180
        return Fx , Fy, Cx, Cy


class kinect_v1Driver(cameradriver):
    def __init__(self,subscriber=True, client=False, topic_rgb = "kinect/image_raw", topic_depth = "kinect/depth/image_raw", node_name="kinect_v1_sub", mode = 0):
        
        self.topic_rgb = topic_rgb
        self.topic_depth = topic_depth

        if mode == 2: # depth
            topic = self.topic_depth
        else: # rgb
            topic = self.topic_rgb

        super().__init__(subscriber, client, topic, node_name)
        self.mode = mode
        self.display_mode = mode
        self.current_frame_rgb = None
        self.current_frame_depth = None

        self.timestamp_rgb = None
        self.timestamp_depth = None

        if mode == 0: # both rgb and depth
            self.init_subscriber()
            self.display_mode = 1 # default display rgb
        else: 
            super().set_mode()

    def init_subscriber(self):
        self.subscription_rgb = self.create_subscription(Image, self.topic_rgb, self.subscribe_rgb, qos_profile)
        self.subscription_depth = self.create_subscription(Image, self.topic_depth, self.subscribe_depth, qos_profile)
        
    def subscribe_rgb(self,msg):
        self.msg_rgb = msg
        self.timestamp_rgb = msg.header.stamp
        self.current_frame_rgb = self.br.imgmsg_to_cv2(self.msg_rgb)
        self.current_frame_rgb = cv2.cvtColor(self.current_frame_rgb, cv2.COLOR_BGR2RGB)
        # self.get_logger().info('Received a RGB frame')

    def subscribe_depth(self,msg):
        self.msg_depth = msg
        self.timestamp_depth = msg.header.stamp
        self.current_frame_depth = self.br.imgmsg_to_cv2(self.msg_depth)
        # self.get_logger().info('Received a Depth frame')

    def get_depth_frame(self):
        return self.current_frame_depth
    
    def get_timestamp(self):
        if self.mode == 2:
            return self.timestamp_depth
        else:
            return self.timestamp_rgb

    def display(self, display_mode=1): # display_mode: 1 for rgb , 2 for depth
        if self.mode == 0:
            self.display_mode = display_mode
        if self.display_mode == 1:
            if self.current_frame_rgb is not None:
                cv2.namedWindow(f'displaying {self.node_name}{self.display_mode}', cv2.WINDOW_NORMAL) 
                cv2.imshow(f'displaying {self.node_name}{self.display_mode}', self.current_frame_rgb)
                self.get_logger().info('displaying')
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    rclpy.shutdown()
            else: 
                self.get_logger().warn("No images recieved to display")
        elif self.display_mode == 2:
            if self.current_frame_depth is not None:
                cv2.namedWindow(f'displaying {self.node_name}', cv2.WINDOW_NORMAL) 
                cv2.imshow(f'displaying {self.node_name}', self.current_frame_depth)
                self.get_logger().info('displaying')
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    rclpy.shutdown()
            else: 
                self.get_logger().warn("No images recieved to display")

    def get_frame(self):
        if self.mode == 0 or self.mode == 1:
            return self.current_frame_rgb
        elif self.mode == 2:
            return self.current_frame_depth

    def get_internsic_values(self):
        Fx =  572.99
        Fy =  542.74
        Cx =  314.65
        Cy =  240.17
        return Fx , Fy, Cx, Cy


class kinect_v2Driver(kinect_v1Driver):
    def __init__(self,subscriber=True, client=False, topic_rgb = "kinect2/image_raw", topic_depth = "kinect2/depth/image_raw", node_name="kinect_v2_sub", mode = 0):
        super().__init__(subscriber, client, topic_rgb, topic_depth, node_name, mode)

