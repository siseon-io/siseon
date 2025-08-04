# app/webcam/loader.py

import logging

from ultralytics import YOLO


def load_model(model_path: str, conf_thres: float) -> YOLO:
    """
    YOLO 모델을 지정된 경로에서 불러오고 confidence threshold를 설정합니다.

    Args:
        model_path (str): 모델 파일 경로
        conf_thres (float): confidence threshold 값

    Returns:
        YOLO: 로드된 YOLO 모델 객체
    """
    model = YOLO(model_path)
    model.conf = conf_thres
    logging.info(f'모델 로드 완료: {model_path}, conf={conf_thres}')
    return model
