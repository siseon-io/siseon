#!/usr/bin/env python3
# app/webcam/main.py

import logging
import cv2
import numpy as np

from utils import init_logging, set_seed, configure_device
from app.webcam.capture import init_video_capture
from app.webcam.loader import load_model, load_pose_model
from app.webcam.postproc import extract_eye_centers, extract_pose_keypoints
from app.webcam.draw import draw_bboxes, draw_pose

from net.sender import send_all

def main(
    model_path: str,
    pose_model_path: str,
    conf_thres: float,
    index: int,
    width: int,
    height: int,
) -> None:
    """
    Initialize models and run real-time gaze + pose detection on webcam feed.

    Args:
        model_path (str): YOLO gaze model file path
        pose_model_path (str): YOLO pose model file path
        conf_thres (float): confidence threshold for gaze model
        index (int): webcam device index
        width (int): frame width
        height (int): frame height
    """
    # 1) 로깅, 시드, 디바이스 설정
    init_logging()
    set_seed(42)
    configure_device(gpu_id=0)

    # 2) 모델 및 카메라 초기화
    gaze_model = load_model(model_path=model_path, conf_thres=conf_thres)
    pose_model = load_pose_model(model_path=pose_model_path)
    cap = init_video_capture(index=index, width=width, height=height)
    logging.info("시작: 'q' 키를 눌러 종료하세요.")

    # 3) 메인 루프
    while True:
        ret, frame = cap.read()
        if not ret:
            logging.error('프레임을 읽을 수 없습니다.')
            break

        # --- Gaze 추론 및 그리기 ---
        gaze_results = gaze_model.predict(source=frame, verbose=False)
        left_eye, right_eye = extract_eye_centers(gaze_results)
        frame = draw_bboxes(frame, gaze_results)

        # --- Pose 추론 및 그리기 ---
        pose_results = pose_model(frame, verbose=False)
        kpts = extract_pose_keypoints(pose_results)
        frame = draw_pose(frame, kpts)

        # --- 데이터 전송 ---
        pose_pts = kpts[:, :2]
        send_all(
            left_eye[0], left_eye[1],
            right_eye[0], right_eye[1],
            pose_pts
        )

        # --- 화면 표시 ---
        cv2.imshow('Webcam Eye & Pose', frame)

        # 종료 키 처리
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 4) 자원 해제
    cap.release()
    cv2.destroyAllWindows()
