import numpy as np
from typing import Any, Tuple

def extract_eye_centers(
    results: Any
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """
    YOLO gaze 결과에서 왼쪽/오른쪽 눈 중심 좌표를 반환합니다.
    results: 모델 추론 결과 리스트
    return: (left_eye, right_eye), 각 (x, y)
    """
    # 1) 모든 박스의 중심 좌표 수집
    centers = []
    for res in results:
        for box in res.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            centers.append(((x1 + x2) / 2, (y1 + y2) / 2))

    # 2) 비어 있으면 기본값으로 (0,0), (0,0) 리턴
    if not centers:
        return (0.0, 0.0), (0.0, 0.0)

    # 3) x 좌표 기준 오름차순 정렬
    centers = sorted(centers, key=lambda p: p[0])

    # 4) 그룹 나누기
    if len(centers) >= 4:
        left_group = centers[:2]
        right_group = centers[2:4]
    elif len(centers) >= 2:
        left_group = [centers[0]]
        right_group = [centers[1]]
    else:
        # 1개만 검출된 경우, 둘 다 같은 점
        left_group = [centers[0]]
        right_group = [centers[0]]

    # 5) 그룹별 평균 계산
    def mean_point(pts):
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    left_eye = mean_point(left_group)
    right_eye = mean_point(right_group)

    return left_eye, right_eye


def extract_pose_keypoints(results: Any) -> np.ndarray:
    """
    YOLO pose 결과에서 keypoint (x,y,conf) 배열을 반환합니다.
    항상 2차원 배열 (M,3)을 반환하여 draw 및 전송에서 unpack 안전 보장
    """
    # 결과 없거나 keypoint가 비어 있으면 (1,3) 제로 배열 리턴
    if not results or len(results[0].keypoints) == 0:
        return np.zeros((1, 3), dtype=float)

    # Keypoints 객체에서 raw tensor 추출
    kp_obj = results[0].keypoints
    # 내부 데이터 텐서를 가져오기. Keypoints.data에는 torch.Tensor일 수 있습니다.
    raw_tensor = getattr(kp_obj, 'data', kp_obj)
    # CPU numpy 배열로 변환
    raw_arr = raw_tensor.cpu().numpy()

    # (1, N, 3) 형태이면 첫 번째 차원만 취해 (N,3)으로 변환
    if raw_arr.ndim == 3:
        kpts = raw_arr[0]
    else:
        kpts = raw_arr

    # 1차원 배열인 경우 2차원 배열로 변환
    kpts = np.atleast_2d(kpts)
    return kpts
