# camera/capture.py

import cv2
import logging

def init_video_capture(index: int, width: int, height: int) -> cv2.VideoCapture:
    """
    OpenCV VideoCapture 초기화
    """
    cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        logging.error(f"Cannot open camera index={index}")
        raise RuntimeError("Webcam open failed")
    return cap
