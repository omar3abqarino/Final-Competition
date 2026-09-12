from camera_drivers.cameraclass import mono
import rclpy

def main():
    rclpy.init(args=None)
    
    # ---- create object ---- #
    mono_server = mono(camera_index = 0, server=True, service="mono_service") # service name is optional

    
    #-----test_server-----#
    rclpy.spin(mono_server)
    mono_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()