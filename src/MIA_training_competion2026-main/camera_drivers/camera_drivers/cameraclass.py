import cv2
import numpy as np
import threading
import rclpy
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image, CompressedImage
from rclpy.node import Node
from srv_pkg.srv import SendImg
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=3
)

class cameraClass(Node): 

    def __init__(self, camera_index = 0, publisher=False, server=False, topic_name="camera_topic", node_name="camera_node", service="camera_service", compressed = False):
        super().__init__(node_name)

        self.topic_name = topic_name
        self.service = service
        self.compressed = compressed

        self.cap = None
        self.frame = None
        self.last_frame = None

        self.camera_index = camera_index
        self.set_camera_index(self.camera_index)

        self.br = CvBridge()
        self.publisherMode=publisher
        self.serverMode=server

        threading.Thread(target=self.capture_frame, daemon=True).start()
            
       #----------set mode------------#
    def set_mode(self): # overwritten in stereo
        if self.publisherMode:
            self.init_publisher()
        if self.serverMode:
            self.init_server()


    # change camera index method
    def set_camera_index(self,camera_index):
        self.camera_index = camera_index
        self.cap = cv2.VideoCapture(self.camera_index)


    # getter method 
    def get_frame(self): # overwritten in stereo
        ret, self.frame = self.cap.read()
        if not self.cap.isOpened():
            self.get_logger().error(f"Camera {self.camera_index} could not be opened.")
        return self.frame
    

    #---------------publisher code---------------#
    # creating publisher node
    def init_publisher(self): # overwritten in stereo
        if self.compressed:
            self.publisher = self.create_publisher(CompressedImage, self.topic_name, qos_profile)
        else:
            self.publisher = self.create_publisher(Image, self.topic_name, qos_profile)
        print("initialized camera_publisher node")

    def publish(self): # overwritten in stereo
    
        if not self.cap.isOpened():
            self.get_logger().error(f"could not open camera {self.camera_index}.")
            return
        
        while self.frame is None:
            self.get_logger().warn("could not capture a frame from camera.")

        if self.last_frame is not None and np.array_equal(self.frame, self.last_frame):
            self.get_logger().warn("repeated a frame.")
        
        self.last_frame = self.frame
            
        if self.compressed:
            compressed_frame = self.compress_frame(self.frame)
            if compressed_frame is not None:
                img_msg = CompressedImage()
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.format = "jpeg"
                img_msg.data = compressed_frame
                self.publisher.publish(img_msg)     
                self.get_logger().info("Published a compressed frame")
                return
        else:
            img_msg = self.br.cv2_to_imgmsg(self.frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(img_msg)
            self.get_logger().info("Published a frame")


   # ----------------server code----------------#
    def init_server(self): # overwritten in stereo
        self.srv = self.create_service(SendImg, self.service, self.send_frame)
        print("initialized camera_server node")

    def send_frame(self, request, response): # overwritten in stereo

        if self.frame is not None:
            response.img = self.br.cv2_to_imgmsg(self.frame, encoding='bgr8')
            self.get_logger().info('reply to frame request')
            return response
        else: 
            self.get_logger().warn("no frame has been recieved")

    def capture_frame(self):

        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame

    def compress_frame(self, frame):
        if frame is not None:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            result, encimg = cv2.imencode('.jpg', frame, encode_param)
            if result:
                return encimg.tobytes()
            else:
                self.get_logger().warn("Failed to compress frame")

    def spin(self):
        rclpy.spin(self)

    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


class mono(cameraClass):
    def __init__(self ,camera_index = 0, publisher=False, server=False, topic_name="mono", node_name="mono_camera_node", service="mono_service", correction = 0, compressed = False):
        super().__init__(camera_index ,publisher, server, topic_name, node_name, service, compressed)
        super().set_mode()

        self.correction = correction

        if self.correction == 1:
            self.mtx = np.array([[227.62354385 , 0.000000, 178.53034818],
                            [0.000000,  226.19965525 ,187.09191809],
                            [0.000000, 0.000000, 1.000000]])
            self.dist = np.array([-0.27687039 , 0.2629094 , -0.03872163, -0.01716333, -0.16969159 ])

    def __distortion_correction(self,frame):
       
        if frame is None:
            return
        
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
        dst = cv2.undistort(frame, self.mtx, self.dist, None, newcameramtx)
        x, y, w, h = roi   
        x, y, w, h = 30,30,300,200
        dst = dst[y:y+h, x:x+w]
        h , w = dst.shape[:2]
        return dst
    
    def get_frame(self): 

        ret, self.frame = self.cap.read()
        if not self.cap.isOpened():
            self.get_logger().error(f"Camera {self.camera_index} could not be opened.")

        if self.correction == 1:
            self.frame=self.__distortion_correction(self.frame)
        return self.frame
    
    def set_mtx(self, mtx):
        self.mtx = mtx

    def set_dist(self, dist):
        self.dist = dist


class stereo(cameraClass):  
    def __init__(self ,camera_index = 0, publisher=False, server=False, topic_name="stereo", node_name="stereo_camera_node", service="stereo_service" ,stereo_i = 0, correction = 1, compressed = False):
        super().__init__(camera_index, publisher, server, topic_name, node_name, service, compressed)
        
        self.stereo_i = stereo_i
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.set_mode()

        self.correction = correction

        if self.correction == 1:
            self.mtx = np.array([[193.87540058231477, 0.000000, 174.71707886808744],
                            [0.000000, 194.9272466095171, 128.32533220178456],
                            [0.000000, 0.000000, 1.000000]])
            self.dist = np.array([-0.3483905945535469, 0.0930933223718445, -0.004300869346806649, -0.006752531206651351, 0.009782105564207623 ])

    def set_mode(self):
        if self.publisherMode:
            self.init_publisher()
        elif self.serverMode:
            self.init_server()

    def init_publisher(self):
        
        if self.compressed:
            if self.stereo_i == 1:
                self.publisher = self.create_publisher(CompressedImage, "left_stereo", qos_profile)
            elif self.stereo_i == 2:
                self.publisher = self.create_publisher(CompressedImage, "right_stereo", qos_profile)
            else:
                self.publisher_l = self.create_publisher(CompressedImage, "left_stereo", qos_profile)
                self.publisher_r = self.create_publisher(CompressedImage, "right_stereo", qos_profile)
        else:
            if self.stereo_i == 1:
                self.publisher = self.create_publisher(Image, "left_stereo", qos_profile)
            elif self.stereo_i == 2:
                self.publisher = self.create_publisher(Image, "right_stereo", qos_profile)
            else:
                self.publisher_l = self.create_publisher(Image, "left_stereo", qos_profile)
                self.publisher_r = self.create_publisher(Image, "right_stereo", qos_profile)
        print("stereo camera publisher inialized")

    def init_server(self): 
        self.srv = self.create_service(SendImg, self.service, self.send_frame)
        print("initialized camera_server node")
        threading.Thread(target=super().capture_frame, daemon=True).start()

    def __get_left_camera(self):
        left_frame = self.frame[:, :self.width // 2]
        return left_frame
    
    def __get_right_camera(self):
        right_frame = self.frame[:, self.width // 2:]
        return right_frame

    def __distortion_correction(self,frame, stereo_i=0):

        if stereo_i == 0:
            left = self.__get_left_camera()
            right = self.__get_right_camera()

            left = self.__distortion_correction(left,1)
            right = self.__distortion_correction(right,2)

            frame = np.hstack((left, right))
            return frame
        
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
        dst = cv2.undistort(frame, self.mtx, self.dist, None, newcameramtx)
        x, y, w, h = roi   
        x, y, w, h = 30,30,280,170
        dst = dst[y:y+h, x:x+w]
        h , w = dst.shape[:2]
        return dst
    
    def get_frame(self):
        ret, self.frame = self.cap.read()
        if self.frame is not None:
            if self.correction:
                self.frame = self.__distortion_correction(self.frame,self.stereo_i)
            else:
                if self.stereo_i == 1:
                    self.frame = self.__get_left_camera
                elif self.stereo_i == 2:
                    self.frame = self.__get_right_camera
            
            return self.frame
    
    def publish(self):
        if not self.cap.isOpened():
                self.get_logger().error(f"Camera {self.camera_index} could not be opened.")
                return

        if self.frame is None:
            self.get_logger().warn("No frame captured from camera.")

        else:
            if self.stereo_i == 0:
                left_frame = self.__get_left_camera()
                right_frame = self.__get_right_camera()

                if self.correction:
                    left_frame = self.__distortion_correction(left_frame,1)
                    right_frame = self.__distortion_correction(right_frame,2)

                if self.compressed:
                    compressed_left = self.compress_frame(left_frame)
                    compressed_right = self.compress_frame(right_frame)

                    if compressed_left is not None:
                        img_msg_l = CompressedImage()
                        img_msg_l.header.stamp = self.get_clock().now().to_msg()
                        img_msg_l.format = "jpeg"
                        img_msg_l.data = compressed_left
                        self.publisher_l.publish(img_msg_l)

                    if compressed_right is not None:
                        img_msg_r = CompressedImage()
                        img_msg_r.header.stamp = self.get_clock().now().to_msg()
                        img_msg_r.format = "jpeg"
                        img_msg_r.data = compressed_right
                        self.publisher_r.publish(img_msg_r)

                    self.get_logger().info(f"Published a compressed frame to left_stereo")
                    self.get_logger().info(f"Published a compressed frame to right_stereo")
                
                else:
                    img_msg_l = self.br.cv2_to_imgmsg(left_frame, encoding='bgr8')
                    img_msg_l.header.stamp = self.get_clock().now().to_msg()
                    self.publisher_l.publish(img_msg_l)

                    img_msg_r = self.br.cv2_to_imgmsg(right_frame, encoding='bgr8')
                    img_msg_r.header.stamp = self.get_clock().now().to_msg()
                    self.publisher_r.publish(img_msg_r)
                    
                    self.get_logger().info(f"Published a frame to left_stereo")
                    self.get_logger().info(f"Published a frame to right_stereo")

            else:
                if self.compressed:
                    compressed_frame = self.compress_frame(self.frame)
                    if compressed_frame is not None:
                        img_msg = CompressedImage()
                        img_msg.header.stamp = self.get_clock().now().to_msg()
                        img_msg.format = "jpeg"
                        img_msg.data = compressed_frame
                        self.publisher.publish(img_msg)
                        self.get_logger().info("Published a compressed frame")
                        return
                else:
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg = self.br.cv2_to_imgmsg(self.frame, encoding='bgr8')
                    self.publisher.publish(img_msg)
                    self.get_logger().info(f"Published a frame to {self.topic_name}")
 
    def send_frame(self, request, response):
        
        if self.frame is not None:
            if self.correction:
                self.frame = self.__distortion_correction(self.frame,self.stereo_i)
            
            response.img = self.br.cv2_to_imgmsg(self.frame, encoding='bgr8')
            self.get_logger().info('reply to frame request')
            return response
        else: 
            self.get_logger().warn("no frame has been recieved")

    def set_mtx(self, mtx):
        self.mtx = mtx

    def set_dist(self, dist):
        self.dist = dist