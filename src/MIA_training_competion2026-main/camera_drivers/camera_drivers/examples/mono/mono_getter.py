from camera_drivers.cameraclass import mono
import rclpy
import cv2

def main():
    rclpy.init(args=None)

    # ---- create object ---- #
    mono_publisher = mono(camera_index=2)


    # ---- test getter ---- #
    while True:

        frame = mono_publisher.get_frame()
        if frame is None:
            continue
        cv2.imshow("mono", frame)
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:  # quit on 'q' or ESC
            break

    cv2.destroyWindow("mono")       


if __name__ == '__main__':
    main()