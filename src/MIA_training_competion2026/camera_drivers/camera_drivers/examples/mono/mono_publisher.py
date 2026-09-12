from camera_drivers.cameraclass import mono
import rclpy
import numpy as np

def main():
    rclpy.init(args=None)

    # if correction is set to 1:
    intrinsic_matrix = np.array([[227.62354385 , 0.000000, 178.53034818],
                            [0.000000,  226.19965525 ,187.09191809],
                            [0.000000, 0.000000, 1.000000]])
    distortion_coefficients = np.array([-0.27687039 , 0.2629094 , -0.03872163, -0.01716333, -0.16969159 ])

    # ---- create object ---- #
    mono_publisher = mono(camera_index=2, publisher=True, topic_name="/mono/image", correction = 0, compressed = False) # setting topic name is optional
    # mono_publisher = mono(camera_index=1, publisher=True, topic_name="/mono", correction = 0, compressed = False) # setting topic name is optional

    mono_publisher.set_mtx(intrinsic_matrix)
    mono_publisher.set_dist(distortion_coefficients)

    # ---- test publisher ---- #
    # while rclpy.ok():
    #     mono_publisher.publish()
    timer = mono_publisher.create_timer(1.0 / 20.0, mono_publisher.publish) # 60 fps

    mono_publisher.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
