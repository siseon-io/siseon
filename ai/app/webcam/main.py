#!/usr/bin/env python3
# app/webcam/main.py

import logging
import cv2

from utils import init_logging, set_seed, configure_device
from app.webcam.capture import init_video_capture
from app.webcam.loader import load_model
from app.webcam.postproc import extract_eye_coords
from app.webcam.draw import draw_bboxes

from pose_estimation.estimator import PoseEstimator


def main(
    model_path: str,
    conf_thres: float,
    index: int,
    width: int,
    height: int,
) -> None:
    """
    웹캠으로부터 프레임을 읽어와 눈 검출 및 포즈 추정 결과를 화면에 표시합니다.

    1) 로깅, 시드, 디바이스 설정
    2) YOLO 모델 및 카메라 초기화
    3) 메인 루프: 프레임 읽기 → 눈 검출 → bounding box 및 포즈 결과 표시
    4) 종료 시 자원 해제

    Args:
        model_path (str): YOLO 모델 파일 경로
        conf_thres (float): 모델 confidence threshold
        index (int): 웹캠 인덱스
        width (int): 프레임 가로 해상도
        height (int): 프레임 세로 해상도
    """
    # 1) 로깅, 시드, 디바이스
    init_logging()
    set_seed(42)
    configure_device(gpu_id=0)

    # 2) 모델 및 카메라 초기화
    model = load_model(model_path=model_path, conf_thres=conf_thres)
    cap = init_video_capture(index=index, width=width, height=height)
    pose_est = PoseEstimator(model_path=model_path.replace('.pt', '-pose.pt'))
    logging.info("시작: 'q' 키를 눌러 종료하세요.")

    # 3) 메인 루프
    while True:
        ret, frame = cap.read()
        if not ret:
            logging.error('프레임을 읽을 수 없습니다.')
            break

        # --- Gaze 추론 및 그리기 ---
        results = model.predict(source=frame, verbose=False)
        coords = extract_eye_coords(results)  # (필요 시 사용)
        draw_bboxes(frame, results)

        # --- Pose 추론 및 그리기 ---
        pose_res = pose_est.detect_pose(frame)
        frame = pose_est.draw_pose(frame, pose_res)

        # --- 화면 표시 ---
        cv2.imshow('Webcam Eye & Pose', frame)

        # 종료 키 처리
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 4) 자원 해제
    cap.release()
    cv2.destroyAllWindows()
