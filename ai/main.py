#!/usr/bin/env python3
import argparse
import yaml
import logging
from pathlib import Path
from ultralytics import YOLO

from utils                           import init_logging, set_seed, configure_device
from gaze_detection.data.download    import download_dataset
from gaze_detection.models.train     import train_model
from gaze_detection.models.evaluate  import evaluate_model
from gaze_detection.viz.visualize    import visualize_samples
from gaze_detection.inference.file   import infer_folder
from gaze_detection.hpo.optimize     import run_hpo

def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-c", "--config",
        default="configs/default.yaml",
        help="Path to YAML config"
    )
    parser.add_argument(
        "-t", "--task",
        choices=["all","hpo","train","eval","viz","inference","webcam"],
        default="all",
        help="Which step to run"
    )
    args = parser.parse_args()

    # ─── 환경 세팅 ─────────────────────────────────────────
    cfg = load_config(args.config)
    init_logging()
    set_seed(cfg["common"]["seed"])
    configure_device(cfg["common"]["gpu_id"])

    # 1) HPO
    if args.task in ("hpo","all"):
        logging.info("=== [TASK] hpo ===")
        run_hpo(cfg)
        if args.task == "hpo":
            return

    # 2) 데이터 다운로드
    gaze_cfg = cfg["gaze"]
    ds_root  = download_dataset(gaze_cfg["data"])
    yaml_p   = Path(ds_root) / "data.yaml"

    model = None

    # 3) 학습
    if args.task in ("train","all"):
        logging.info("=== [TASK] train ===")
        # HPO 결과 불러오기
        hpo_file   = Path(cfg["common"]["paths"]["output_root"]) / "hpo_best_params.yaml"
        hpo_params = yaml.safe_load(open(hpo_file)) if hpo_file.exists() else {}
        model = train_model(cfg["common"], gaze_cfg, yaml_p, **hpo_params)

        # train 후 best.pt 경로를 inference 설정에 반영
        project_dir = Path(cfg["common"]["paths"]["output_root"]) / "train"
        train_name  = f"{gaze_cfg['data']['project_name']}_v{gaze_cfg['data']['version']}"
        new_w       = project_dir / train_name / "weights" / "best.pt"
        cfg["gaze"]["inference"]["weights"] = str(new_w)
        logging.info(f"Updated inference weights to: {new_w}")

        if args.task == "train":
            return

    # 4) 이미 학습된 모델 로드
    if model is None and args.task in ("eval","viz","inference","all"):
        weights = gaze_cfg["inference"]["weights"]
        logging.info(f"Loading model from weights: {weights}")
        model = YOLO(weights)

    # 5) 평가
    if args.task in ("eval","all"):
        logging.info("=== [TASK] eval ===")
        evaluate_model(model, gaze_cfg, yaml_p, split="val")
        evaluate_model(model, gaze_cfg, yaml_p, split="test")
        if args.task == "eval":
            return

    # 6) 시각화 샘플
    if args.task in ("viz","all"):
        logging.info("=== [TASK] viz ===")
        viz_out = Path(cfg["common"]["paths"]["output_root"]) / "gaze" / "viz"
        visualize_samples(model, ds_root/"test"/"images", output_dir=viz_out, num=5)
        if args.task == "viz":
            return

    # 7) 폴더 단위 Inference
    if args.task in ("inference","all"):
        logging.info("=== [TASK] inference ===")
        inf_out = Path(cfg["common"]["paths"]["output_root"]) / "gaze" / "inference"
        infer_folder(
            gaze_cfg["inference"]["weights"],
            ds_root/"test"/"images",
            inf_out,
            gaze_cfg["inference"]["conf_thresh"]
        )
        if args.task == "inference":
            return

    # 8) 실시간 Webcam Inference
    if args.task in ("webcam","all"):
        logging.info("=== [TASK] webcam ===")
        from gaze_detection.inference.webcam.main import main as run_webcam
        run_webcam(
            model_path=cfg["gaze"]["inference"]["weights"],
            conf_thres=cfg["gaze"]["inference"]["conf_thresh"],
            index=cfg["gaze"]["inference"]["webcam_index"],
            width=cfg["gaze"]["inference"]["cam_width"],
            height=cfg["gaze"]["inference"]["cam_height"]
        )
        if args.task == "webcam":
            return

if __name__ == "__main__":
    main()
