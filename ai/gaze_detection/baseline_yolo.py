'''
YOLOv11 기반 눈(eye) 검출 모델 학습 및 평가 스크립트
'''

import os
import random
import logging
from pathlib import Path

import torch
import pandas as pd
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from roboflow import Roboflow
from ultralytics import YOLO


def set_seed(seed: int = 42) -> None:
    """재현성을 위한 랜덤 시드 설정"""
    random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def configure_device(gpu_id: int = 0) -> None:
    """
    CUDA 디바이스 설정
    Args:
        gpu_id (int): 사용할 GPU 번호. 음수인 경우 CPU 사용.
    """
    os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
    if gpu_id >= 0:
        os.environ["CUDA_VISIBLE_DEVICES"] = str(gpu_id)
    else:
        os.environ["CUDA_VISIBLE_DEVICES"] = ""
    logging.info(f"CUDA 디바이스 개수: {torch.cuda.device_count()}")
    if torch.cuda.is_available():
        logging.info(f"CUDA 디바이스 이름: {torch.cuda.get_device_name(0)}")


def download_dataset(api_key: str, workspace: str, project_name: str, version: int,
                     format: str = "yolov11") -> Path:
    """
    Roboflow에서 데이터셋 다운로드
    Returns:
        다운로드된 데이터셋 경로
    """
    rf = Roboflow(api_key=api_key)
    project = rf.workspace(workspace).project(project_name)
    dataset = project.version(version).download(format)
    return Path(dataset.location)


def train_model(yaml_path: Path, output_dir: Path, model_name: str = "yolo11n.pt",
                epochs: int = 100, img_size: int = 416, batch_size: int = 16,
                workers: int = 4, device: str = "0") -> YOLO:
    """
    YOLO 모델 학습
    Returns:
        학습된 YOLO 모델 객체
    """
    model = YOLO(model_name)
    model.train(
        data=str(yaml_path),
        epochs=epochs,
        imgsz=img_size,
        batch=batch_size,
        workers=workers,
        device=device,
        name=output_dir.name,
        project=str(output_dir.parent),
        exist_ok=True
    )
    return model


def evaluate_model(model: YOLO, yaml_path: Path, split: str = "val") -> pd.DataFrame:
    """
    검증 또는 테스트 데이터셋으로 모델 평가
    Returns:
        평가 결과를 담은 DataFrame
    """
    result = model.val(data=str(yaml_path), split=split)
    return result

def visualize_predictions(model: YOLO, image_dir: Path, num_samples: int = 5) -> None:
    """
    임의의 이미지 샘플에 대해 예측 결과 시각화
    """
    images = list(image_dir.glob("*.*"))
    samples = random.sample(images, min(num_samples, len(images)))
    for img_path in samples:
        img = Image.open(img_path)
        results = model(str(img_path))
        boxes = results[0].boxes

        fig, ax = plt.subplots()
        ax.imshow(img)

        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0].cpu().numpy())
            label = results[0].names[cls]
            rect = patches.Rectangle((x1, y1), x2 - x1, y2 - y1,
                                     linewidth=2, edgecolor="yellow", facecolor="none")
            ax.add_patch(rect)
            ax.text(x1, y1 - 5, label, fontsize=10,
                    backgroundcolor="black", color="yellow")

        ax.axis("off")
        plt.show()


def main():
    logging.basicConfig(level=logging.INFO)

    # 재현성 및 디바이스 설정
    set_seed(42)
    configure_device(gpu_id=0)

    # 데이터셋 다운로드 및 경로 설정
    dataset_root = download_dataset(
        api_key="MF8Wd7JxbRUZSmQTC9fw",
        workspace="eye-annotations-yolo-to-voc",
        project_name="eye-detection-kso3d",
        version=4,
    )
    yaml_path = dataset_root / "data.yaml"
    logging.info(f"Dataset root: {dataset_root}")

    # 모델 학습
    output_dir = Path("runs/train/eye_kso3d_v4")
    model = train_model(
        yaml_path=yaml_path,
        output_dir=output_dir,
        model_name="yolo11n.pt",
        epochs=100,
        img_size=416,
        batch_size=16,
        workers=4,
        device="0",
    )

    # 평가
    val_results = evaluate_model(model, yaml_path, split="val")
    logging.info(f"Validation results: {val_results}")
    test_results = evaluate_model(model, yaml_path, split="test")
    logging.info(f"Test results: {test_results}")

    # 학습 결과 시각화
    img = Image.open(output_dir / "results.png")
    img.show()

    # 예측 결과 시각화
    visualize_predictions(model, dataset_root / "test" / "images", num_samples=5)


if __name__ == "__main__":
    main()
