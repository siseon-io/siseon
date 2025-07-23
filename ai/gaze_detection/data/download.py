# gaze_detection/data/download.py

import logging
from pathlib import Path
from roboflow import Roboflow
import yaml

def download_dataset(cfg: dict) -> Path:
    rf = Roboflow(api_key=cfg["api_key"])
    proj = rf.workspace(cfg["workspace"])\
             .project(cfg["project_name"])
    ds   = proj.version(cfg["version"])\
             .download(cfg["data_fmt"])
    logging.info(f"Dataset downloaded to {ds.location}")
    return Path(ds.location)
