from camera_drivers.cameradriver import kinect_v1Driver
import rclpy

def main():
    rclpy.init(args=None)
    # ---- create object ---- #
    kinect_v1_rgb = kinect_v1Driver(mode=1) # mode 1 (rgb) / mode 2 (depth)
    kinect_v1_depth = kinect_v1Driver(mode=2)

    while rclpy.ok():
        
        kinect_v1_rgb.spin(timeout=0.01)
        kinect_v1_rgb.display()

        kinect_v1_depth.spin(timeout=0.01)
        kinect_v1_depth.display()
        
    kinect_v1_rgb.destroy_node()
    kinect_v1_depth.destroy_node()
    rclpy.shutdown()