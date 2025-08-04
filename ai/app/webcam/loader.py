# app/webcam/loader.py

import logging
from ultralytics import YOLO

def load_model(model_path: str, conf_thres: float) -> YOLO:
    """
    YOLO 기반 gaze(눈) 검출 모델을 로드하고 confidence threshold를 설정합니다.

    Args:
        model_path (str): .pt 파일 경로
        conf_thres (float): confidence threshold
    """
    model = YOLO(model_path)
    model.conf = conf_thres
    logging.info(f'Gaze 모델 로드 완료: {model_path}, conf={conf_thres}')
    return model

def load_pose_model(model_path: str) -> YOLO:
    """
    YOLO 기반 pose(포즈) 추정 모델을 로드합니다.

    Args:
        model_path (str): .pt 파일 경로
    """
    model = YOLO(model_path)
    logging.info(f'Pose 모델 로드 완료: {model_path}')
    return model
