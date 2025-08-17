# gaze_detection/hpo/optimize.py

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Dict, Mapping

import optuna
import yaml
from optuna.trial import Trial
from ultralytics import YOLO

from gaze_detection.data.download import download_dataset

__all__ = ['run_hpo']

LOGGER = logging.getLogger(__name__)


def _objective(
    trial: Trial,
    model_name: str,
    img_size: int,
    batch_size: int,
    yaml_path: Path,
    hpo_cfg: Mapping[str, Any],
    output_root: Path,
) -> float:
    """
    Optuna trial 목표 함수로 mAP50을 최대화합니다.

    Args:
        trial (Trial): Optuna trial 객체
        model_name (str): 모델 파일명
        img_size (int): 입력 이미지 크기
        batch_size (int): 배치 크기
        yaml_path (Path): 데이터셋 설정 파일 경로
        hpo_cfg (Mapping[str, Any]): HPO 설정 딕셔너리
        output_root (Path): 결과 출력 루트 경로

    Returns:
        float: mAP50 지표 값
    """
    # 탐색할 하이퍼파라미터 설정
    lr0 = trial.suggest_float('lr0', 1e-4, 1e-1, log=True)
    lrf = trial.suggest_float('lrf', 0.1, 0.5)
    momentum = trial.suggest_float('momentum', 0.8, 0.99)
    weight_decay = trial.suggest_float('weight_decay', 1e-5, 1e-2, log=True)

    warmup_epochs = trial.suggest_int('warmup_epochs', 0, 10)
    warmup_momentum = trial.suggest_float('warmup_momentum', 0.5, 0.9)
    warmup_bias_lr = trial.suggest_float('warmup_bias_lr', 1e-5, 1e-2, log=True)

    box_gain = trial.suggest_float('box', 0.02, 0.2)
    cls_gain = trial.suggest_float('cls', 0.5, 2.0)
    kobj_gain = trial.suggest_float('kobj', 0.5, 2.0)
    iou_gain = trial.suggest_float('iou', 0.5, 0.9)

    hsv_h = trial.suggest_float('hsv_h', 0.0, 0.05)
    hsv_s = trial.suggest_float('hsv_s', 0.0, 1.0)
    hsv_v = trial.suggest_float('hsv_v', 0.0, 1.0)
    degrees = trial.suggest_float('degrees', 0.0, 10.0)
    translate = trial.suggest_float('translate', 0.0, 0.2)
    scale = trial.suggest_float('scale', 0.3, 1.0)
    shear = trial.suggest_float('shear', 0.0, 5.0)
    mosaic = trial.suggest_float('mosaic', 0.0, 1.0)
    mixup = trial.suggest_float('mixup', 0.0, 0.5)

    model = YOLO(model_name)
    epochs = int(hpo_cfg.get('trial_epochs', 1))

    model.train(
        data=str(yaml_path),
        epochs=epochs,
        lr0=lr0,
        lrf=lrf,
        momentum=momentum,
        weight_decay=weight_decay,
        warmup_epochs=warmup_epochs,
        warmup_momentum=warmup_momentum,
        warmup_bias_lr=warmup_bias_lr,
        box=box_gain,
        cls=cls_gain,
        kobj=kobj_gain,
        iou=iou_gain,
        hsv_h=hsv_h,
        hsv_s=hsv_s,
        hsv_v=hsv_v,
        degrees=degrees,
        translate=translate,
        scale=scale,
        shear=shear,
        mosaic=mosaic,
        mixup=mixup,
        imgsz=img_size,
        batch=batch_size,
        device='0',
        project=str(output_root),
        name=f'hpo_trial_{trial.number}',
        exist_ok=True,
    )

    metrics = model.val(data=str(yaml_path), split='val')
    map50 = metrics.box.map
    LOGGER.info('[Trial %d] mAP50=%.4f', trial.number, map50)
    return map50


def run_hpo(cfg: Dict[str, Any]) -> None:
    """
    YAML 설정(dict)을 받아 Optuna 하이퍼파라미터 최적화를 실행합니다.

    Args:
        cfg (Dict[str, Any]): 전체 설정 딕셔너리
    """
    gaze_cfg: Mapping[str, Any] = cfg['gaze']
    model_cfg: Mapping[str, Any] = cfg['model']
    hpo_cfg: Mapping[str, Any] = cfg.get('hpo', {})
    common_cfg: Mapping[str, Any] = cfg['common']

    # 데이터셋 준비
    datasets_root = Path(common_cfg['paths']['output_root']) / 'datasets'
    datasets_root.mkdir(parents=True, exist_ok=True)
    dest_dir = datasets_root / gaze_cfg['data']['project_name']
    _, yaml_path = download_dataset(gaze_cfg['data'], dest_dir=dest_dir)

    # 결과 출력 디렉터리 준비
    output_root = Path(common_cfg['paths']['output_root']) / 'hpo'
    output_root.mkdir(parents=True, exist_ok=True)

    # Pruner 설정
    pruner = None
    if 'pruner' in hpo_cfg:
        pr_cfg = hpo_cfg['pruner']
        pruner = getattr(optuna.pruners, f"{pr_cfg['type'].capitalize()}Pruner")(
            n_warmup_steps=int(pr_cfg.get('warmup_steps', 5))
        )

    study = optuna.create_study(direction='maximize', pruner=pruner)
    study.optimize(
        lambda t: _objective(
            t,
            model_cfg['name'],
            model_cfg['img_size'],
            model_cfg['batch_size'],
            yaml_path,
            hpo_cfg,
            output_root,
        ),
        n_trials=int(hpo_cfg.get('n_trials', 20)),
        timeout=hpo_cfg.get('timeout'),
    )

    best = study.best_trial
    LOGGER.info('🏆 Best trial #%d → %s', best.number, best.params)

    out_yaml = output_root / 'hpo_best_params.yaml'
    with open(out_yaml, 'w', encoding='utf-8') as f:
        yaml.safe_dump(best.params, f, allow_unicode=True)
    LOGGER.info('✓ 최적 파라미터 저장: %s', out_yaml)
