from camera_drivers.cameradriver import monoDriver
import rclpy

def main():
    rclpy.init(args=None)
    # ---- create object ---- #
    mono_subscriber = monoDriver(subscriber=True, topic="/mono/image", compressed = False) # defualt topic name is "mono"

    while rclpy.ok():
        
        mono_subscriber.spin(timeout=0.01)
        mono_subscriber.display()
        
    mono_subscriber.destroy_node()
    rclpy.shutdown()