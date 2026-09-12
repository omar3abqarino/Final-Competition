from camera_drivers.cameradriver import kinect_v2Driver
import rclpy
import cv2

def main():
    rclpy.init(args=None)
    # ---- create object ---- #
    kinect_v2_subscriber = kinect_v2Driver(mode=0) # defualt: mode 0 (rgb & depth) / mode 1 (rgb) / mode 2 (depth)

    while rclpy.ok():
        
        kinect_v2_subscriber.spin(timeout=0.01)
        kinect_v2_subscriber.display(display_mode=1) # defualt: display_mode 1 (rgb) / display_mode 2 (depth)
        kinect_v2_subscriber.display(display_mode=2) 
        
    kinect_v2_subscriber.destroy_node()
    rclpy.shutdown()