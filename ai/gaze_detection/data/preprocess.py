# gaze_detection/data/preprocess.py

import logging
from pathlib import Path

def convert_to_voc(dataset_root: Path):
    """
    YOLO → VOC 형식으로 변환 (필요 시)
    """
    # TODO: 실제 전처리 코드 작성
    logging.info(f"Converting {dataset_root} to VOC format…")
