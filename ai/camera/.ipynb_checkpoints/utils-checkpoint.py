# camera/utils.py

import numpy as np

def resize_frame(frame, max_dim=800):
    """
    프레임 비율 유지하며 최대 길이를 max_dim으로 조정
    """
    h, w = frame.shape[:2]
    scale = max_dim / max(h, w)
    new_size = (int(w * scale), int(h * scale))
    return cv2.resize(frame, new_size)
