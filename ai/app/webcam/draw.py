# app/webcam/draw.py

import cv2
import numpy as np

def draw_bboxes(frame: np.ndarray, results) -> np.ndarray:
    """
    Gaze 검출 결과의 bounding box를 그립니다.
    """
    for res in results:
        for box in res.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return frame

def draw_pose(frame: np.ndarray, keypoints: np.ndarray) -> np.ndarray:
    """
    Pose 추정 keypoint를 그립니다.
    """
    for x, y, c in keypoints:
        if c > 0.3:  # confidence threshold
            cv2.circle(frame, (int(x), int(y)), 3, (0, 0, 255), -1)
    # TODO: skeleton 연결 선 그리기
    return frame
