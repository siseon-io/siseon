# gaze_detection/inference/file.py

import logging
from pathlib import Path
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from ultralytics import YOLO

def infer_folder(model_path: str, image_dir: Path, output_dir: Path, conf: float):
    model = YOLO(model_path)
    model.conf = conf
    output_dir.mkdir(parents=True, exist_ok=True)

    for img_path in image_dir.glob("*.*"):
        logging.info(f"Inferencing {img_path.name}")
        results = model.predict(source=str(img_path), verbose=False)
        img = Image.open(img_path)
        fig, ax = plt.subplots()
        ax.imshow(img)
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0].cpu().numpy())
            label = results[0].names[cls]
            rect = patches.Rectangle((x1, y1), x2 - x1, y2 - y1,
                                     linewidth=2, edgecolor="yellow", facecolor="none")
            ax.add_patch(rect)
            ax.text(x1, y1-5, label, fontsize=10, backgroundcolor="black", color="yellow")
        ax.axis("off")
        save_path = output_dir / img_path.name
        fig.savefig(save_path, bbox_inches="tight", pad_inches=0)
        plt.close(fig)
        logging.info(f"Saved to {save_path}")
