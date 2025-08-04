# app/webcam/capture.py

import cv2
import logging

def init_video_capture(index: int, width: int, height: int) -> cv2.VideoCapture:
    """
    OpenCV VideoCapture 객체를 초기화합니다.

    Args:
        index (int): 사용할 웹캠 인덱스
        width (int): 프레임 가로 해상도
        height (int): 프레임 세로 해상도

    Returns:
        cv2.VideoCapture: 초기화된 VideoCapture 객체

    Raises:
        RuntimeError: 웹캠을 열지 못했을 때 발생
    """
    cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    if not cap.isOpened():
        logging.error(f'웹캠 열기 실패 index={index}')
        raise RuntimeError('웹캠을 열 수 없습니다.')

    return cap
