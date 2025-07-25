# gaze_detection/hpo/optimize.py

import logging
import optuna
from pathlib import Path
from ultralytics import YOLO

from gaze_detection.data.download import download_dataset


def _get_gaze_cfg(cfg: dict) -> dict:
    """	cfg에서 gaze 섹션만 분리합니다. """
    if "gaze" in cfg:
        return cfg["gaze"]
    # legacy 지원이 필요하다면 여기에 추가
    raise KeyError("No 'gaze' section in config")


def objective(
    trial: optuna.Trial,
    gaze_cfg: dict,
    hpo_cfg: dict,
    yaml_path: Path,
    output_root: str
) -> float:
    """
    한 trial에 대해 모델을 학습(소수 epoch) → val mAP50 계산 후 리턴
    """
    # 1) HPO 대상 파라미터
    lr0          = trial.suggest_float("lr0", 1e-4, 1e-1, log=True)
    lrf          = trial.suggest_float("lrf", 0.1, 0.5)
    momentum     = trial.suggest_float("momentum", 0.8, 0.99)
    weight_decay = trial.suggest_float("weight_decay", 1e-5, 1e-2, log=True)

    warmup_epochs   = trial.suggest_int("warmup_epochs", 0, 10)
    warmup_momentum = trial.suggest_float("warmup_momentum", 0.5, 0.9)
    warmup_bias_lr  = trial.suggest_float("warmup_bias_lr", 1e-5, 1e-2, log=True)

    box_gain = trial.suggest_float("box", 0.02, 0.2)
    cls_gain = trial.suggest_float("cls", 0.5, 2.0)
    kobj     = trial.suggest_float("kobj", 0.5, 2.0)
    iou      = trial.suggest_float("iou", 0.5, 0.9)

    hsv_h     = trial.suggest_float("hsv_h", 0.0, 0.05)
    hsv_s     = trial.suggest_float("hsv_s", 0.0, 1.0)
    hsv_v     = trial.suggest_float("hsv_v", 0.0, 1.0)
    degrees   = trial.suggest_float("degrees", 0.0, 10.0)
    translate = trial.suggest_float("translate", 0.0, 0.2)
    scale     = trial.suggest_float("scale", 0.3, 1.0)
    shear     = trial.suggest_float("shear", 0.0, 5.0)
    mosaic    = trial.suggest_float("mosaic", 0.0, 1.0)
    mixup     = trial.suggest_float("mixup", 0.0, 0.5)

    # 2) trial당 학습 epoch 수는 cfg["hpo"]["trial_epochs"]
    epochs = hpo_cfg.get("trial_epochs", 1)

    # 3) 모델 학습
    model = YOLO(gaze_cfg["model"]["name"])
    model.train(
        data=str(yaml_path),
        epochs=epochs,
        lr0=lr0, lrf=lrf,
        momentum=momentum,
        weight_decay=weight_decay,
        warmup_epochs=warmup_epochs,
        warmup_momentum=warmup_momentum,
        warmup_bias_lr=warmup_bias_lr,
        box=box_gain, cls=cls_gain, kobj=kobj, iou=iou,
        hsv_h=hsv_h, hsv_s=hsv_s, hsv_v=hsv_v,
        degrees=degrees, translate=translate, scale=scale,
        shear=shear, mosaic=mosaic, mixup=mixup,
        imgsz=gaze_cfg["model"]["img_size"],
        batch=gaze_cfg["model"]["batch_size"],
        device="0",
        name=f"hpo_trial_{trial.number}",
        project=output_root,
        exist_ok=True
    )

    # 4) 검증 결과 취득
    metrics = model.val(data=str(yaml_path), split="val")
    mAP50   = metrics.box.map
    logging.info(
        f"[Trial {trial.number}] epochs={epochs}  "
        f"lr0={lr0:.2e}, momentum={momentum:.3f}, wd={weight_decay:.2e}  "
        f"mAP50={mAP50:.4f}"
    )
    return mAP50


def run_hpo(cfg: dict):
    """
    cfg에서 gaze_cfg와 hpo_cfg를 분리해 Optuna 스터디 실행
    """
    gaze_cfg = _get_gaze_cfg(cfg)
    hpo_cfg  = cfg.get("hpo", {})
    common   = cfg["common"]
    output_root = common["paths"]["output_root"]

    # 데이터 다운로드
    ds_root = download_dataset(gaze_cfg["data"])
    yaml_p  = Path(ds_root) / "data.yaml"

    # Pruner 설정 (optional)
    pruner = None
    if "pruner" in hpo_cfg:
        pr = hpo_cfg["pruner"]
        pruner = getattr(optuna.pruners, f"{pr['type'].capitalize()}Pruner")(
            n_warmup_steps=pr.get("warmup_steps", 5)
        )

    study = optuna.create_study(direction="maximize", pruner=pruner)
    study.optimize(
        lambda t: objective(t, gaze_cfg, hpo_cfg, yaml_p, output_root),
        n_trials=hpo_cfg.get("n_trials", 20),
        timeout=hpo_cfg.get("timeout", None)
    )

    best = study.best_trial
    logging.info(f"Best trial #{best.number} → params: {best.params}")
    out_file = Path(output_root) / "hpo_best_params.yaml"
    with open(out_file, "w") as f:
        import yaml as _yaml
        _yaml.dump(best.params, f)
    logging.info(f"Saved best hyperparameters to {out_file}")
