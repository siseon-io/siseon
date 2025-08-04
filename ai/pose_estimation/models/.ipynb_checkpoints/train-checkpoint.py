# /ai/pose_estimation/models/train.py
"""
YOLOv11n‑Pose (상체 10관절) 파인튜닝 스크립트
> python train.py --data ../data/FLIC_yolo/flic_upperbody.yaml
"""
import argparse
from pathlib import Path
from ultralytics import YOLO

def parse_opt():
    parser = argparse.ArgumentParser()
    # 필수 인자
    parser.add_argument('--data', type=str,
                        default=str(Path(__file__).resolve().parents[1] /
                                    'data/FLIC_yolo/flic_upperbody.yaml'),
                        help='dataset YAML')
    # 선택 인자
    parser.add_argument('--weights', type=str, default='yolo11n-pose.pt',
                        help='pretrained YOLOv11n‑Pose 가중치(.pt) or .yaml')
    parser.add_argument('--epochs', type=int, default=80)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--device', type=str, default='0')     # 'cpu' or '0,1'
    parser.add_argument('--freeze', type=int, default=10,      # backbone 일부 동결
                        help='freeze first n layers; 0 = none')
    return parser.parse_args()

def main(opt):
    # 1. 모델 로드 (가중치 파일이면 뼈대+파라미터 동시 로드)
    model = YOLO(opt.weights)

    # 2. 학습 – Ultralytics가 kpt_shape 17→10 불일치 시
    #    최종 Head 레이어를 자동 재초기화해 줍니다.
    model.train(
        data  = opt.data,
        epochs= opt.epochs,
        imgsz = opt.imgsz,
        batch = opt.batch,
        device= opt.device,
        freeze= opt.freeze,   # backbone 경량 fine‑tuning
        optimizer='AdamW',
        project='runs/pose',  # 기본 저장 디렉터리
        name='flic_upperbody_y11n',
        verbose=True
    )

    # 3. 최적 모델(.pt) → Jetson Nano용 TensorRT FP16 엔진까지 내보내기
    best_pt = Path(model.trainer.best)
    print(f"\n✅ best weights saved to: {best_pt}")
    engine_path = best_pt.with_suffix('.engine')
    model = YOLO(best_pt)
    model.export(format='engine', half=True, device=opt.device,
                 imgsz=opt.imgsz, optimize=True, dynamic=False,
                 workspace=2, save_dir=best_pt.parent)  # *.engine 옆에 저장
    print(f"🚀 TensorRT FP16 engine saved to: {engine_path}")

if __name__ == "__main__":
    opt = parse_opt()
    main(opt)
