# pose_estimation/models/train.py

from __future__ import annotations

import logging
from pathlib import Path
from typing import Mapping, Any

from ultralytics import YOLO

__all__ = ['train_model']


def train_model(
    common_cfg: Mapping[str, Any],
    pose_cfg: Mapping[str, Any],
    yaml_path: Path,
) -> YOLO:
    """
    YOLOv11n-Pose 모델을 freeze해서 하단(head) 레이어만 fine-tuning 합니다.

    Args:
        common_cfg (Mapping[str, Any]): 'common' 설정 딕셔너리
        pose_cfg   (Mapping[str, Any]): 'pose' 설정 딕셔너리
        yaml_path  (Path): Roboflow 스타일 데이터셋 YAML 경로
    Returns:
        YOLO: 학습된 모델 객체
    """
    # 1) 모델 로드
    weights = pose_cfg['weights']
    model = YOLO(weights)
    logging.info("✅ Pose 모델 로드: %s", weights)

    # 2) 출력 경로 및 이름 설정
    root = Path(common_cfg['paths']['output_root'])
    project_dir = root / 'train'
    train_name = pose_cfg.get('run_name', pose_cfg.get('project_name', 'pose'))

    # 3) 학습 인자 구성 (freeze만)
    train_args: dict[str, Any] = {
        'data': str(yaml_path),
        'epochs': pose_cfg['epochs'],
        'imgsz': pose_cfg['imgsz'],
        'batch': pose_cfg['batch'],
        'device': str(common_cfg.get('gpu_id', 0)),
        'freeze': pose_cfg['freeze'],            # backbone 상위 N개 레이어 동결
        'optimizer': pose_cfg.get('optimizer', 'AdamW'),
        'project': str(project_dir),
        'name': train_name,
        'exist_ok': True,
        'verbose': True,
    }

    logging.info("▶️ Pose 학습 시작: %s", train_args)
    model.train(**train_args)
    logging.info("✅ Pose 학습 종료 (%s)", train_name)

    return model
