# gaze_detection/inference/file.py

from __future__ import annotations

import logging
from pathlib import Path
from typing import Final, Union

import matplotlib.patches as patches
import matplotlib.pyplot as plt
from PIL import Image
from ultralytics import YOLO

__all__: Final = ['infer_folder']


def infer_folder(
    model_path: Union[str, Path],
    *,
    image_folder: Path,
    output_dir: Path,
    conf_thresh: float = 0.25,
) -> None:
    """
    지정한 폴더 내 모든 이미지를 YOLO 모델로 추론하고 시각화 결과를 저장합니다.

    Args:
        model_path (Union[str, Path]): 학습된 YOLO 가중치(.pt) 경로
        image_folder (Path): 추론 대상 이미지가 들어있는 폴더 경로
        output_dir (Path): 시각화 결과를 저장할 폴더 경로
        conf_thresh (float, optional): confidence threshold (기본값: 0.25)
    """
    model = YOLO(str(model_path))
    model.conf = conf_thresh

    output_dir.mkdir(parents=True, exist_ok=True)
    img_paths = sorted(image_folder.glob('*.*'))

    for idx, img_path in enumerate(img_paths, 1):
        logging.info('[%d/%d] 예측 중: %s', idx, len(img_paths), img_path.name)

        results = model.predict(source=str(img_path), verbose=False)
        img = Image.open(img_path).convert('RGB')

        fig, ax = plt.subplots(figsize=(8, 6))
        ax.imshow(img)

        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0].cpu().numpy())
            label = results[0].names[cls]

            rect = patches.Rectangle(
                (x1, y1),
                x2 - x1,
                y2 - y1,
                linewidth=2,
                edgecolor='yellow',
                facecolor='none',
            )
            ax.add_patch(rect)
            ax.text(
                x1,
                y1 - 5,
                label,
                fontsize=10,
                backgroundcolor='black',
                color='yellow',
            )

        ax.axis('off')

        save_path = output_dir / f'{img_path.stem}_viz.png'
        fig.savefig(save_path, bbox_inches='tight', pad_inches=0)
        plt.close(fig)

        logging.info('저장 완료: %s', save_path)
