# /ai/pose_estimation/models/eval.py
"""
학습된 YOLOv11‑Pose 모델 평가
> python eval.py --weights runs_pose/flic_upperbody_y11n/weights/best.pt
"""
import argparse
from ultralytics import YOLO
from pathlib import Path

def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, required=True,
                        help='*.pt model to evaluate')
    parser.add_argument('--data', type=str,
                        default=str(Path(__file__).resolve().parents[1] /
                                    'data/FLIC_yolo/flic_upperbody.yaml'))
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--batch', type=int, default=32)
    parser.add_argument('--device', type=str, default='0')
    return parser.parse_args()

def main(opt):
    model   = YOLO(opt.weights)
    metrics = model.val(
        data  = opt.data,
        imgsz = opt.imgsz,
        batch = opt.batch,
        device= opt.device)
    print("\n📊 Validation metrics:")
    for k, v in metrics.results_dict.items():
        print(f"{k:<20}: {v:.4f}")

if __name__ == "__main__":
    main(parse_opt())
