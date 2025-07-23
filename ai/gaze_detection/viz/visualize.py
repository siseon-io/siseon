# gaze_detection/viz/visualize.py

import random
from pathlib import Path
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import logging

def visualize_samples(model, image_dir: Path, output_dir: Path = None, num: int = 5):
    """
    샘플 예측 결과를 저장하거나 화면에 표시합니다.
    
    Args:
        model: 학습된 YOLO 모델 객체
        image_dir (Path): 테스트 이미지 폴더
        output_dir (Path, optional): 지정하면 여기로 PNG 파일로 저장, None이면 plt.show()
        num (int): 샘플 개수
    """
    images = list(image_dir.glob("*.*"))
    samples = random.sample(images, min(len(images), num))

    if output_dir:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        logging.info(f"Saving visualizations to {output_dir}")

    for idx, img_path in enumerate(samples, 1):
        img = Image.open(img_path).convert("RGB")
        res = model(str(img_path))
        
        fig, ax = plt.subplots()
        ax.imshow(img)
        for box in res[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0].cpu().numpy())
            label = res[0].names[cls]
            rect = patches.Rectangle(
                (x1, y1),
                x2 - x1,
                y2 - y1,
                linewidth=2,
                edgecolor="yellow",
                facecolor="none"
            )
            ax.add_patch(rect)
            ax.text(
                x1, y1 - 5,
                label, fontsize=10,
                backgroundcolor="black",
                color="yellow"
            )
        ax.axis("off")

        if output_dir:
            save_path = output_dir / f"{img_path.stem}_viz.png"
            fig.savefig(save_path, bbox_inches="tight", pad_inches=0)
            logging.info(f"[{idx}/{len(samples)}] Saved: {save_path}")
            plt.close(fig)
        else:
            plt.show()
