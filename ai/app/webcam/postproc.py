# app/webcam/postproc.py

import numpy as np
from typing import Any

def extract_eye_coords(results: Any) -> list[tuple[float, float]]:
    """
    YOLO gaze 결과에서 눈 영역 박스 중심 좌표 리스트를 반환합니다.
    """
    coords = []
    for res in results:
        for box in res.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            coords.append(((x1 + x2) / 2, (y1 + y2) / 2))
    return coords

def extract_pose_keypoints(results: Any) -> np.ndarray:
    """
    YOLO pose 결과에서 keypoint (x,y,conf) 배열을 반환합니다.
    """
    if not results or len(results[0].keypoints) == 0:
        return np.empty((0, 3))
    kpts = results[0].keypoints.cpu().numpy()[0]  # shape (N,3)
    return kpts
