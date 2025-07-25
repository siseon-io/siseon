# gaze_detection/inference/webcam/loader.py

from ultralytics import YOLO
import logging

def load_model(model_path: str, conf_thres: float):
    model = YOLO(model_path)
    model.conf = conf_thres
    logging.info(f"Model loaded: {model_path}, conf={conf_thres}")
    return model
