#!/usr/bin/env python3
# build_flic_yolo.py
"""
FLIC (examples.mat + images/) → Ultralytics YOLOv11‑Pose (L) 데이터셋 자동 변환

출력 폴더 구조
FLIC_yolo/
  images/
    train/*.jpg
    val/*.jpg
  labels/
    train/*.txt  # class, xc, yc, w, h + 10*(x,y,v) = 35개 값
    val/*.txt
  flic_upperbody.yaml
"""

import os
import shutil
import random
import yaml
import numpy as np
import cv2
from tqdm import tqdm
from scipy.io import loadmat
import argparse

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--src-dir',   type=str, default='.', help='examples.mat 과 images/ 폴더가 있는 경로')
    p.add_argument('--dst-dir',   type=str, default='FLIC_yolo', help='변환된 데이터셋 출력 경로')
    p.add_argument('--val-ratio', type=float, default=0.2, help='검증셋 비율 (0~1)')
    p.add_argument('--seed',      type=int,   default=42, help='랜덤 시드')
    p.add_argument('--pad-px',    type=int,   default=20, help='bbox padding (px)')
    return p.parse_args()

def yolo_label_line(bbox, kpts_norm, vis):
    """
    YOLOv11-Pose 라벨 한 줄 (class + bbox + 10*(x,y,v))
    => 총 1 + 4 + 10*3 = 35개 숫자
    """
    # bbox 값 클리핑 (0~1 사이)
    xc, yc, w, h = bbox
    xc, yc, w, h = [max(0.0, min(1.0, x)) for x in (xc, yc, w, h)]

    parts = [f"{0:d}",
             f"{xc:.6f}", f"{yc:.6f}", f"{w:.6f}", f"{h:.6f}"]

    for (x, y), v in zip(kpts_norm, vis):
        # keypoint 도 클리핑
        x = max(0.0, min(1.0, x))
        y = max(0.0, min(1.0, y))
        parts += [f"{x:.6f}", f"{y:.6f}", f"{v:d}"]

    return " ".join(parts) + "\n"

def process_split(examples, indices, split, args):
    img_out = os.path.join(args.dst_dir, 'images', split)
    lbl_out = os.path.join(args.dst_dir, 'labels', split)
    os.makedirs(img_out, exist_ok=True)
    os.makedirs(lbl_out, exist_ok=True)

    for i in tqdm(indices, desc=f"[{split}]"):
        ex = examples[i]
        rel_path = ex.filepath
        src_img = os.path.join(args.src_dir, 'images', rel_path)
        dst_img = os.path.join(img_out, rel_path)
        os.makedirs(os.path.dirname(dst_img), exist_ok=True)
        shutil.copy2(src_img, dst_img)

        # keypoints & bbox
        kpts_px = ex.coords.T        # shape (10,2)
        img = cv2.imread(src_img)
        h, w = img.shape[:2]
        x1, y1, x2, y2 = ex.torsobox
        x1 = max(0, x1 - args.pad_px)
        y1 = max(0, y1 - args.pad_px)
        x2 = min(w, x2 + args.pad_px)
        y2 = min(h, y2 + args.pad_px)
        bbox = (
            (x1 + x2) / 2 / w,
            (y1 + y2) / 2 / h,
            (x2 - x1) / w,
            (y2 - y1) / h,
        )
        kpts_norm = kpts_px / np.array([w, h])
        vis = np.ones((10,), dtype=int)

        rel_txt = rel_path.rsplit('.',1)[0] + '.txt'
        with open(os.path.join(lbl_out, rel_txt), 'w') as f:
            f.write(yolo_label_line(bbox, kpts_norm, vis))

if __name__ == '__main__':
    args = parse_args()
    random.seed(args.seed)

    # .mat 파일 로드
    mat      = loadmat(os.path.join(args.src_dir, 'examples.mat'),
                       struct_as_record=False,
                       squeeze_me=True)
    examples = mat['examples']

    # train/val split 인덱스 생성
    idx_all = list(range(len(examples)))
    random.shuffle(idx_all)
    split_i = int(len(idx_all) * (1 - args.val_ratio))
    idx_train, idx_val = idx_all[:split_i], idx_all[split_i:]

    # 출력 디렉토리 구조 생성
    for d in ['images/train','images/val','labels/train','labels/val']:
        os.makedirs(os.path.join(args.dst_dir, d), exist_ok=True)

    # 데이터 복사 및 라벨 생성
    process_split(examples, idx_train, 'train', args)
    process_split(examples, idx_val,   'val',   args)

    # YAML 파일 쓰기
    yaml_dict = {
        'path': args.dst_dir.replace('\\','/'),
        'train': 'images/train',
        'val':   'images/val',
        'labels': {
            'train': 'labels/train',
            'val':   'labels/val'
        },
        'names': {0: 'person'},
        'kpt_shape': [10, 3],
        'flip_idx': [1,4, 2,5, 3,6, 7,8],
    }
    yaml_path = os.path.join(args.dst_dir, 'flic_upperbody.yaml')
    with open(yaml_path, 'w') as f:
        yaml.safe_dump(yaml_dict, f, sort_keys=False)

    print(f"\n✅  변환 완료!  '{args.dst_dir}' 폴더를 확인하세요.")
    print(f"   YAML: {yaml_path}")
    print("   예시) yolo pose train model=yolo11l-pose.pt data=flic_upperbody.yaml epochs=80 imgsz=640")
