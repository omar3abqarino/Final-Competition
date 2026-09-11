from camera_drivers.cameradriver import kinect_v2Driver
import rclpy

def main(): ######## NOT TESTED ############
    rclpy.init(args=None)
    # ---- create object ---- #
    kinect_v2_rgb = kinect_v2Driver(mode=1) # mode 1 (rgb) / mode 2 (depth)
    kinect_v2_depth = kinect_v2Driver(mode=2)

    while rclpy.ok():
        
        kinect_v2_rgb.spin(timeout=0.02)
        kinect_v2_rgb.display()

        kinect_v2_depth.spin(timeout=0.02)
        kinect_v2_depth.display()
        
    kinect_v2_rgb.destroy_node()
    kinect_v2_depth.destroy_node()
    rclpy.shutdown()