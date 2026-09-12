from camera_drivers.cameradriver import monoDriver
import rclpy
import cv2

def main():
    rclpy.init(args=None)
    
    # ---- create object ---- #
    mono_client = monoDriver(client=True, service="mono_service") # defualt service name is mono_service

    #-----test_client-----#
    response = mono_client.send_request()
    mono_client.get_logger().info(f"Response received: {response}")
    
    cv2.imshow("mono example", response)
    cv2.waitKey(0)

    rclpy.spin(mono_client)
    mono_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()