from __future__ import annotations

import logging
import random
from pathlib import Path
from typing import Final, Optional

import matplotlib.patches as patches
import matplotlib.pyplot as plt
from PIL import Image
from ultralytics import YOLO

__all__: Final = ['visualize_samples']


def visualize_samples(
    model: YOLO,
    image_dir: Path,
    *,
    output_dir: Optional[Path] = None,
    num: int = 5,
) -> None:
    """
    무작위 이미지 몇 장을 예측하여 시각화 PNG를 저장하거나 화면에 표시합니다.

    Args:
        model (YOLO): 학습된 YOLO 모델 객체
        image_dir (Path): 예측할 이미지가 담긴 디렉터리
        output_dir (Optional[Path], optional): 지정 시 PNG로 저장, None이면 plt.show() (기본값: None)
        num (int, optional): 시각화할 샘플 개수 (기본값: 5)
    """
    images = sorted(image_dir.glob('*.*'))
    if not images:
        logging.warning('이미지 폴더가 비어 있습니다: %s', image_dir)
        return

    samples = random.sample(images, k=min(len(images), num))

    if output_dir:
        output_dir.mkdir(parents=True, exist_ok=True)
        logging.info('시각화 저장 경로: %s', output_dir)

    for idx, img_path in enumerate(samples, 1):
        # 이미지 불러오기 및 예측
        img = Image.open(img_path).convert('RGB')
        results = model(str(img_path), verbose=False)

        # 시각화
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.imshow(img)
        ax.axis('off')

        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls_id = int(box.cls[0].cpu().numpy())
            label = results[0].names[cls_id]

            # 바운딩 박스 그리기
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
                color='yellow',
                backgroundcolor='black',
            )

        # 저장 또는 표시
        if output_dir:
            out_path = output_dir / f'sample_{idx}.png'
            fig.savefig(out_path, bbox_inches='tight', pad_inches=0.1)
            plt.close(fig)
            logging.info('시각화 이미지 저장됨: %s', out_path)
        else:
            plt.show()

    logging.info('총 %d개의 샘플 시각화 완료', len(samples))
