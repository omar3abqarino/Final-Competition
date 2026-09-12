from camera_drivers.cameraclass import stereo
import rclpy
import cv2

def main():
    rclpy.init(args=None)

    # ---- create object ---- #
    stereo_cam = stereo(camera_index=2, stereo_i=0)

    # ---- test getter ---- #
    while True:

        frame = stereo_cam.get_frame()
        if frame is None:
            continue

        cv2.imshow("stereo", frame)
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:  # quit on 'q' or ESC
            break

    cv2.destroyWindow("stereo")       


if __name__ == '__main__':
    main()