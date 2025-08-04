# gaze_detection/models/evaluate.py

from __future__ import annotations

import logging
import os
from typing import Final, Union

from ultralytics import YOLO

__all__: Final = ['evaluate_model']


def evaluate_model(
    model: YOLO,
    yaml_path: Union[str, bytes, os.PathLike],
    *,
    split: str = 'val',
):
    """
    검증 또는 테스트 세트에서 mAP 지표를 계산하고 로그로 남깁니다.

    Args:
        model (YOLO): 학습된 YOLO 모델 객체
        yaml_path (str | bytes | os.PathLike): Roboflow `data.yaml` 파일 경로
        split (str, optional): 'val' 또는 'test' 세트 중 선택 (기본값: 'val')

    Returns:
        ultralytics.utils.metrics.DetMetrics: 평가 결과 객체
    """
    logging.info('평가 시작 (split=%s)', split)
    metrics = model.val(data=str(yaml_path), split=split)

    map50 = metrics.box.map50
    map5095 = metrics.box.map
    logging.info(
        '%s mAP@0.5: %.4f | mAP@0.5:0.95: %.4f',
        split,
        map50,
        map5095,
    )
    return metrics
