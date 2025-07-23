# gaze_detection/models/train.py

import logging
from pathlib import Path
from ultralytics import YOLO

def train_model(common_cfg: dict, gaze_cfg: dict, yaml_path: Path, **hpo_overrides) -> YOLO:
    """
    YOLO 모델 학습

    Args:
        common_cfg (dict): common 섹션의 설정 (gpu_id, paths.output_root 등)
        gaze_cfg   (dict): gaze 섹션의 설정 (data, model, inference)
        yaml_path  (Path): Roboflow data.yaml 경로
        **hpo_overrides: Optuna 등에서 찾아낸 하이퍼파라미터 (lr0, momentum 등)

    Returns:
        YOLO: 학습된 모델 객체
    """
    # 1) 모델 초기화
    model = YOLO(gaze_cfg["model"]["name"])

    # 2) 출력 경로 및 이름 설정
    project_dir = Path(common_cfg["paths"]["output_root"]) / "train"
    train_name  = f"{gaze_cfg['data']['project_name']}_v{gaze_cfg['data']['version']}"

    # 3) 기본 학습 인자 준비
    train_args = {
        "data":     str(yaml_path),
        "epochs":   gaze_cfg["model"]["epochs"],
        "imgsz":    gaze_cfg["model"]["img_size"],
        "batch":    gaze_cfg["model"]["batch_size"],
        "workers":  gaze_cfg["model"]["workers"],
        # configure_device() 로 이미 CUDA_VISIBLE_DEVICES 셋팅됨 -> 내부 device="0" 사용
        "device":   "0",
        "name":     train_name,
        "project":  str(project_dir),
        "exist_ok": True,
    }

    # 4) HPO 파라미터 덮어쓰기
    if hpo_overrides:
        logging.info(f"Applying HPO overrides: {hpo_overrides}")
        train_args.update(hpo_overrides)

    logging.info(f"Training with args: {train_args}")

    # 5) 학습 실행
    model.train(**train_args)
    return model
