#!/usr/bin/env python3
# main.py

from __future__ import annotations

import argparse
import logging
import subprocess
from pathlib import Path
from typing import Any, Dict

import yaml
from ultralytics import YOLO
from utils import configure_device, init_logging, set_seed

# Gaze Detection imports
from gaze_detection.data.download import download_dataset as gaze_download_dataset
from gaze_detection.hpo.optimize import run_hpo as gaze_run_hpo
from gaze_detection.models.train import train_model as gaze_train_model
from gaze_detection.models.evaluate import evaluate_model as gaze_evaluate_model
from gaze_detection.inference.file import infer_folder as gaze_infer_folder
from gaze_detection.viz.visualize import visualize_samples as gaze_visualize_samples

LOGGER = logging.getLogger(__name__)


def _parse_args() -> argparse.Namespace:
    """
    커맨드라인 인자를 파싱합니다.

    Returns:
        argparse.Namespace: 파싱된 인자
    """
    parser = argparse.ArgumentParser(description='Unified Gaze & Pose CLI')
    parser.add_argument(
        '-c', '--config', default='configs/default.yaml',
        help='Path to config YAML'
    )
    parser.add_argument(
        '-d', '--domain', choices=['gaze', 'pose'],
        default='gaze', help='Which pipeline to run'
    )
    parser.add_argument(
        '-t', '--task',
        choices=['pretrain', 'hpo', 'train', 'eval', 'viz', 'inference', 'webcam', 'all'],
        default='train', help='Stage to run'
    )
    return parser.parse_args()


def _load_config(path: str | Path) -> Dict[str, Any]:
    """
    YAML 설정 파일을 로드합니다.

    Args:
        path (str | Path): YAML 파일 경로

    Returns:
        dict: 설정 딕셔너리
    """
    with open(path, 'r', encoding='utf-8') as fp:
        return yaml.safe_load(fp)


def main() -> None:
    """
    CLI 진입점 함수
    """
    args = _parse_args()
    cfg = _load_config(args.config)

    common_cfg = cfg.get('common', {})
    gaze_cfg = cfg.get('gaze', {})
    pose_cfg = cfg.get('pose', {})
    model_cfg = cfg.get('model', {})
    infer_cfg = cfg.get('inference', {})

    init_logging()
    set_seed(common_cfg.get('seed', 42))
    configure_device(common_cfg.get('gpu_id', 0))

    output_root = Path(common_cfg.get('paths', {}).get('output_root', 'runs'))
    output_root.mkdir(parents=True, exist_ok=True)
    datasets_dir = output_root / 'datasets'
    datasets_dir.mkdir(parents=True, exist_ok=True)

    # WEBCAM: run gaze then pose demos
    if args.task == 'webcam':
        # Gaze webcam
        from app.webcam.main import main as gaze_webcam
        LOGGER.info('▶️ [GAZE WEBCAM]')
        gaze_webcam(
            model_path=infer_cfg['weights'],
            conf_thres=infer_cfg['conf_thresh'],
            index=infer_cfg['webcam_index'],
            width=infer_cfg['cam_width'],
            height=infer_cfg['cam_height'],
        )
        # Pose webcam (script)
        script = Path(__file__).resolve().parent / 'pose_estimation/app/webcam/main.py'
        cmd = ['python', str(script), '--cfg', args.config]
        LOGGER.info('▶️ [POSE WEBCAM] %s', ' '.join(cmd))
        subprocess.run(cmd, check=True)
        return

    # Gaze pipeline
    if args.domain == 'gaze':
        # PRETRAIN
        if args.task == 'pretrain':
            pre_cfg = gaze_cfg['pretrain']
            ds_root, yaml_p = gaze_download_dataset(pre_cfg, dest_dir=datasets_dir / 'pretrain')
            params = {
                'data': pre_cfg,
                'model': {
                    'name': model_cfg['name'],
                    'epochs': pre_cfg['epochs'],
                    'img_size': pre_cfg['img_size'],
                    'batch_size': pre_cfg['batch_size'],
                    'workers': pre_cfg['workers'],
                },
            }
            LOGGER.info('▶️ [GAZE PRETRAIN]')
            gaze_train_model(common_cfg, params, yaml_p)
            return

        # HPO
        if args.task == 'hpo':
            LOGGER.info('▶️ [GAZE HPO]')
            gaze_run_hpo(cfg)
            return

        # TRAIN
        if args.task == 'train':
            data_cfg = gaze_cfg['data']
            ds_root, yaml_p = gaze_download_dataset(data_cfg, dest_dir=datasets_dir / data_cfg['project_name'])
            pre_name = f"{gaze_cfg['pretrain']['project_name']}_v{gaze_cfg['pretrain']['version']}"
            pre_w = output_root / 'train' / pre_name / 'weights' / 'best.pt'
            init_w = str(pre_w) if pre_w.exists() else model_cfg['name']
            LOGGER.info('▶️ [GAZE TRAIN] Using weights: %s', init_w)
            hpo_file = output_root / 'hpo' / 'hpo_best_params.yaml'
            overrides: Dict[str, Any] = {}
            if hpo_file.exists():
                overrides = yaml.safe_load(hpo_file.read_text(encoding='utf-8')) or {}
                LOGGER.info('Applying HPO overrides: %s', overrides)
            ft_cfg = {
                'data': data_cfg,
                'model': {
                    'name': init_w,
                    'epochs': model_cfg['epochs'],
                    'img_size': model_cfg['img_size'],
                    'batch_size': model_cfg['batch_size'],
                    'workers': model_cfg['workers'],
                },
            }
            gaze_train_model(common_cfg, ft_cfg, yaml_p, **overrides)
            return

        # EVAL
        if args.task == 'eval':
            ds_root = datasets_dir / gaze_cfg['data']['project_name']
            yaml_p = ds_root / 'data.yaml'
            model = YOLO(infer_cfg['weights'])
            LOGGER.info('▶️ [GAZE EVAL]')
            gaze_evaluate_model(model, yaml_p, split='val')
            gaze_evaluate_model(model, yaml_p, split='test')
            return

        # VIZ / ALL
        if args.task in ('viz', 'all'):
            ds_root = datasets_dir / gaze_cfg['data']['project_name']
            viz_out = output_root / 'gaze' / 'viz'
            viz_out.mkdir(parents=True, exist_ok=True)
            model = YOLO(infer_cfg['weights'])
            LOGGER.info('▶️ [GAZE VIZ]')
            gaze_visualize_samples(model, ds_root / 'test' / 'images', output_dir=viz_out, num=5)
            if args.task == 'viz':
                return

        # INFERENCE / ALL
        if args.task in ('inference', 'all'):
            ds_root = datasets_dir / gaze_cfg['data']['project_name']
            inf_out = output_root / 'gaze' / 'inference'
            inf_out.mkdir(parents=True, exist_ok=True)
            LOGGER.info('▶️ [GAZE INFERENCE]')
            gaze_infer_folder(
                model_path=infer_cfg['weights'],
                image_folder=ds_root / 'test' / 'images',
                output_dir=inf_out,
                conf_thresh=infer_cfg['conf_thresh']
            )
            return

        LOGGER.error('Unknown gaze task: %s', args.task)
        return

    # Pose pipeline (via scripts)
    if args.domain == 'pose':
        # TRAIN
        if args.task == 'train':
            script = Path(__file__).resolve().parent / 'pose_estimation/models/train.py'
            cmd = [
                'python', str(script),
                '--data', pose_cfg['dataset_yaml'],
                '--weights', pose_cfg['weights'],
                '--epochs', str(pose_cfg['epochs']),
                '--imgsz', str(pose_cfg['imgsz']),
                '--batch', str(pose_cfg['batch']),
                '--freeze', str(pose_cfg['freeze'])
            ]
            LOGGER.info('▶️ [POSE TRAIN] %s', ' '.join(cmd))
            subprocess.run(cmd, check=True)
            return

        # INFERENCE
        if args.task == 'inference':
            script = Path(__file__).resolve().parent / 'pose_estimation/models/infer.py'
            cmd = ['python', str(script), '--cfg', args.config]
            LOGGER.info('▶️ [POSE INFERENCE] %s', ' '.join(cmd))
            subprocess.run(cmd, check=True)
            return

        LOGGER.error("Task '%s' not implemented for pose", args.task)
        return

    LOGGER.error('Unknown domain: %s', args.domain)


if __name__ == '__main__':
    main()
