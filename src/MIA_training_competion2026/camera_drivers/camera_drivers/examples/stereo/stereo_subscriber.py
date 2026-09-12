from camera_drivers.cameradriver import stereoDriver
import rclpy

def main():
    rclpy.init(args=None)

    # ---- create object ---- #
    # you have to set the topic names here to avoid conflict
    # default topic is "stereo"
    left_subscriber = stereoDriver(subscriber=True, topic="left_stereo", compressed=False) 
    right_subscriber = stereoDriver(subscriber=True, topic="right_stereo", compressed=False)

    while rclpy.ok():
        
        left_subscriber.spin(timeout=0.01)
        left_subscriber.display()

        right_subscriber.spin(timeout=0.01)
        right_subscriber.display()
        
    right_subscriber.destroy_node()
    rclpy.shutdown()