from camera_drivers.cameradriver import stereoDriver
import rclpy
import cv2

def main():
    rclpy.init(args=None)
    
    # ---- create object ---- #
    stereo_client = stereoDriver(client=True, service="stereo_service") # defualt service name is stereo_service

    #-----test_client-----#
    response = stereo_client.send_request()
    stereo_client.get_logger().info(f"Response received: {response}")

    cv2.imshow("stereo example", response)
    cv2.waitKey(0)

    rclpy.spin(stereo_client)
    stereo_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()