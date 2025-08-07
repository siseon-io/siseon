import socket
import json
from typing import Sequence, Tuple, Union
import numpy as np


def _send_udp(data: dict, ip: str, port: int) -> None:
    """
    Helper to send a dict as JSON via UDP.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    msg = json.dumps(data).encode('utf-8')
    sock.sendto(msg, (ip, port))
    sock.close()

def send_all(
    lepupil_x, lepupil_y, repupil_x, repupil_y,
    keypoints_xy, ip="192.168.30.121", port=30080
):
    # 1) numpy array 로 변환
    pts = np.array(keypoints_xy)

    # 2) (1, N, 2) 형태라면 (N, 2) 로 평탄화
    if pts.ndim == 3 and pts.shape[0] == 1:
        pts = pts[0]

    # 3) (N, 2) 가 맞는지 확인
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError(f"Invalid shape for pose keypoints: {pts.shape}")

    # 4) 이제 안전하게 unpack
    pose_xy = [[float(x), float(y)] for x, y in pts]

    data = {
        "lepupil_x": float(lepupil_x),
        "lepupil_y": float(lepupil_y),
        "repupil_x": float(repupil_x),
        "repupil_y": float(repupil_y),
        "pose_xy": pose_xy
    }
    _send_udp(data, ip, port)
