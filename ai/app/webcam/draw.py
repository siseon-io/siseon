# app/webcam/draw.py

import cv2
import numpy as np


def draw_bboxes(frame: np.ndarray, results) -> None:
    """
    검출된 박스와 클래스 이름을 원본 프레임에 그립니다.

    Args:
        frame (np.ndarray): 원본 이미지 프레임 (BGR)
        results: 박스와 클래스 정보를 가진 객체 (예: YOLO 결과)
    """
    for box in results[0].boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
        cls_id = int(box.cls[0].cpu().numpy())
        label = results[0].names[cls_id]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.putText(
            frame,
            label,
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1
        )
