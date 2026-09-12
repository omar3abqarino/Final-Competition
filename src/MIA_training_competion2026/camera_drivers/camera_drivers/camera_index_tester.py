import cv2
import argparse
import time

def test_cameras(start=0, end=10, wait_ms=30):
    """
    Try to open cameras with indices in range(start, end).
    Shows each camera in a window until 'n' (next) or 'q' (quit) is pressed.
    """
    for idx in range(start, end):
        print(f"Trying camera index {idx}...")
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)  # Linux-friendly backend
        if not cap.isOpened():
            print(f"  index {idx}: cannot open")
            cap.release()
            continue

        # warm up and grab a frame
        ok, frame = False, None
        for _ in range(5):
            ok, frame = cap.read()
            if ok and frame is not None:
                break
            time.sleep(0.05)

        if not ok or frame is None:
            print(f"  index {idx}: opened but no frame")
            cap.release()
            continue

        win_name = f"Camera {idx} (n: next, q: quit)"
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.imshow(win_name, frame)

        while True:
            key = cv2.waitKey(wait_ms) & 0xFF
            if key == ord('n'):  # next camera
                break
            if key == ord('q') or key == 27:  # quit on 'q' or ESC
                cap.release()
                cv2.destroyAllWindows()
                return
            # try to fetch and show live frames
            ok, frame = cap.read()
            if ok and frame is not None:
                cv2.imshow(win_name, frame)

        cap.release()
        cv2.destroyWindow(win_name)
        print(f"  index {idx}: closed")

def main():
    parser = argparse.ArgumentParser(description="Loop through camera indices and open them one by one")
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--end", type=int, default=10, help="one past the last index to try")
    parser.add_argument("--wait-ms", type=int, default=30, help="cv2.waitKey delay in ms")
    args = parser.parse_args()
    test_cameras(args.start, args.end, args.wait_ms)

if __name__ == "__main__":
    main()