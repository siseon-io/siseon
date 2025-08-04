# gaze_detection/models/train.py

from __future__ import annotations

import logging
from pathlib import Path
from typing import Final, Mapping, Any

from ultralytics import YOLO

__all__: Final = ['train_model']


def train_model(
    common_cfg: Mapping[str, Any],
    gaze_cfg: Mapping[str, Any],
    yaml_path: Path,
    **hpo_overrides,
) -> YOLO:
    """
    YOLO 모델을 학습하고 학습된 모델 객체를 반환합니다.

    Args:
        common_cfg (Mapping[str, Any]): 전체 설정의 'common' 서브딕셔너리
        gaze_cfg (Mapping[str, Any]): 전체 설정의 'gaze' 또는 파인튜닝 서브딕셔너리
        yaml_path (Path): Roboflow `data.yaml` 파일 경로
        **hpo_overrides: Optuna 등으로 탐색한 하이퍼파라미터 덮어쓰기

    Returns:
        YOLO: 학습 완료된 YOLO 모델 객체
    """
    # 1) 모델 초기화
    model_name = gaze_cfg['model']['name']
    model = YOLO(model_name)
    logging.info('모델 초기화: %s', model_name)

    # 2) 출력 디렉터리 설정
    project_dir = Path(common_cfg['paths']['output_root']) / 'train'
    train_name = (
        f"{gaze_cfg['data']['project_name']}_v{gaze_cfg['data']['version']}"
    )

    # 3) 기본 학습 인자 구성
    train_args = {
        'data': str(yaml_path),
        'epochs': gaze_cfg['model']['epochs'],
        'imgsz': gaze_cfg['model']['img_size'],
        'batch': gaze_cfg['model']['batch_size'],
        'workers': gaze_cfg['model']['workers'],
        'device': '0',  # `configure_device()`로 CUDA_VISIBLE_DEVICES 이미 설정
        'name': train_name,
        'project': str(project_dir),
        'exist_ok': True,
    }

    # 4) HPO 파라미터 덮어쓰기
    if hpo_overrides:
        logging.info('HPO 덮어쓰기: %s', hpo_overrides)
        train_args.update(hpo_overrides)

    logging.info('학습 인자: %s', train_args)

    # 5) 학습 실행
    model.train(**train_args)
    logging.info('학습 종료 (%s)', train_name)
    return model
