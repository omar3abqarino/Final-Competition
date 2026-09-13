import cv2
import numpy as np


def detect_boxes(frame):
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    kernel = np.ones((3, 3), np.uint8)
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    boxes = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 500:
            continue

        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)

        if 4 <= len(approx) <= 6:
            x, y, w, h = cv2.boundingRect(approx)
        else:
            
            rect = cv2.minAreaRect(c)
            (_, _), (rw, rh), _ = rect
            if rw <= 0 or rh <= 0:
                continue
            x, y, w, h = cv2.boundingRect(c)

        aspect = w / float(h)
        if 0.6 < aspect < 1.8:
            boxes.append((x, y, w, h, c))
    return boxes