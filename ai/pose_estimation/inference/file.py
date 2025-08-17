# pose_estimation/inference/file.py

from __future__ import annotations

import logging
from pathlib import Path
from typing import Final, Union

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
    지정한 폴더 내 모든 이미지를 YOLO pose 모델로 추론하고
    keypoint 시각화 결과를 저장합니다.

    Args:
        model_path (Union[str, Path]): 학습된 YOLO pose 가중치(.pt) 경로
        image_folder (Path): 추론 대상 이미지가 들어있는 폴더 경로
        output_dir (Path): 시각화 결과를 저장할 폴더 경로
        conf_thresh (float, optional): confidence threshold (기본값: 0.25)
    """
    # 모델 로드 및 설정
    model = YOLO(str(model_path))
    model.conf = conf_thresh  

    output_dir.mkdir(parents=True, exist_ok=True)
    img_paths = sorted(image_folder.glob('*.*'))

    for idx, img_path in enumerate(img_paths, start=1):
        logging.info('[%d/%d] Pose 예측 중: %s', idx, len(img_paths), img_path.name)

        # 추론
        results = model.predict(source=str(img_path), verbose=False)
        
        # 원본 이미지 불러와 RGB로 변환
        img = Image.open(img_path).convert('RGB')

        # Matplotlib으로 시각화
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.imshow(img)

        # keypoints 그리기
        res = results[0]
        if hasattr(res, 'keypoints') and len(res.keypoints) > 0:
            # (N_keypoints, 3) 배열: x, y, confidence
            kpts = res.keypoints.cpu().numpy()[0]
            xs, ys, cs = kpts[:, 0], kpts[:, 1], kpts[:, 2]
            # confidence threshold를 넘는 점만 표시
            for x, y, c in zip(xs, ys, cs):
                if c >= conf_thresh:
                    ax.plot(x, y, 'ro', markersize=4)

        ax.axis('off')

        # 저장
        save_path = output_dir / f'{img_path.stem}_pose_viz.png'
        fig.savefig(save_path, bbox_inches='tight', pad_inches=0)
        plt.close(fig)

        logging.info('저장 완료: %s', save_path)
