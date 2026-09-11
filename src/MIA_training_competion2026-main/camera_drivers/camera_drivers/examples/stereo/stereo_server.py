from camera_drivers.cameraclass import stereo
import rclpy

def main():
    rclpy.init(args=None)
    
    # ---- create object ---- #
    stereo_server = stereo(camera_index = 2, server=True, stereo_i=0 , service="stereo_service") # service name is optional
    
    #-----test_server-----#
    rclpy.spin(stereo_server)
    stereo_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()