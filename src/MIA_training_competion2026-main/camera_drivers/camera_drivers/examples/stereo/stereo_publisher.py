from camera_drivers.cameraclass import stereo
import rclpy
import numpy as np

def main():
    rclpy.init(args=None)

    # if correction is set to 1:
    intrinsic_matrix = np.array([[193.87540058231477, 0.000000, 174.71707886808744],
                            [0.000000, 194.9272466095171, 128.32533220178456],
                            [0.000000, 0.000000, 1.000000]])
    distortion_coefficients = np.array([-0.3483905945535469, 0.0930933223718445, -0.004300869346806649, -0.006752531206651351, 0.009782105564207623 ])

    # ---- create object ---- #
    stereo_publisher = stereo(camera_index = 2, publisher=True, stereo_i=0, compressed= False, correction=1) # i = 0 : publish to 2 topics left_stereo & right_stereo

    stereo_publisher.set_mtx(intrinsic_matrix)
    stereo_publisher.set_dist(distortion_coefficients)

    # ---- test publisher ---- #
    # while rclpy.ok():
    #     stereo_publisher.publish()
    timer = stereo_publisher.create_timer(1.0 / 30.0, stereo_publisher.publish) # 30 fps

    rclpy.spin(stereo_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()