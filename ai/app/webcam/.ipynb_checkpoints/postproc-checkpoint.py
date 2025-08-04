# app/webcam/postproc.py

from typing import Any, Dict


def extract_eye_coords(results: Any) -> Dict[str, float]:
    """
    YOLO 결과에서 왼쪽/오른쪽 눈 영역(클래스 0, 1)과
    동공(클래스 2, 3)의 중심 좌표를 추출하여 반환합니다.

    클래스 ID:
        0: lefteye_region
        1: righteye_region
        2: lefteye_pupil
        3: righteye_pupil

    Args:
        results (Any): YOLO 모델의 추론 결과

    Returns:
        dict: 각 눈과 동공의 중심 좌표 (값이 없으면 None)
              {
                  'lefteye_x': float or None,
                  'lefteye_y': float or None,
                  'righteye_x': float or None,
                  'righteye_y': float or None,
                  'lepupil_x': float or None,
                  'lepupil_y': float or None,
                  'repupil_x': float or None,
                  'repupil_y': float or None,
              }
    """
    coords = {
        'lefteye_x': None, 'lefteye_y': None,
        'righteye_x': None, 'righteye_y': None,
        'lepupil_x': None, 'lepupil_y': None,
        'repupil_x': None, 'repupil_y': None,
    }

    # results가 리스트/튜플인 경우 첫 번째 요소만 사용
    res = results[0] if isinstance(results, (list, tuple)) else results

    # 박스가 없을 경우 바로 반환
    if not hasattr(res, 'boxes') or len(res.boxes) == 0:
        return coords

    # 박스 정보 추출 (tensor → numpy)
    xyxy = res.boxes.xyxy.cpu().numpy()  # shape (N, 4)
    cls_ids = res.boxes.cls.cpu().numpy().astype(int).flatten()  # shape (N,)

    # 각 박스를 순회하며 클래스별로 중심좌표 저장
    for (x1, y1, x2, y2), cid in zip(xyxy, cls_ids):
        cx, cy = float((x1 + x2) / 2), float((y1 + y2) / 2)
        if cid == 0:  # 왼쪽 눈 영역
            coords['lefteye_x'], coords['lefteye_y'] = cx, cy
        elif cid == 1:  # 오른쪽 눈 영역
            coords['righteye_x'], coords['righteye_y'] = cx, cy
        elif cid == 2:  # 왼쪽 동공
            coords['lepupil_x'], coords['lepupil_y'] = cx, cy
        elif cid == 3:  # 오른쪽 동공
            coords['repupil_x'], coords['repupil_y'] = cx, cy

    return coords
