from camera_drivers.cameradriver import kinect_v1Driver
import rclpy
import cv2

def main():
    rclpy.init(args=None)
    # ---- create object ---- #
    kinect_v1_subscriber = kinect_v1Driver(mode=0) # defualt: mode 0 (rgb & depth) / mode 1 (rgb) / mode 2 (depth)

    while rclpy.ok():
        
        kinect_v1_subscriber.spin(timeout=0.01)
        kinect_v1_subscriber.display(display_mode=1) # defualt: display_mode 1 (rgb) / display_mode 2 (depth)
        kinect_v1_subscriber.display(display_mode=2) 
        
    kinect_v1_subscriber.destroy_node()
    rclpy.shutdown()