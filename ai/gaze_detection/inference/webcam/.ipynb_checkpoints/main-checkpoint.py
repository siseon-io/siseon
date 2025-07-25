#!/usr/bin/env python3
import logging
import cv2

from camera.capture                          import init_video_capture
from gaze_detection.inference.webcam.loader   import load_model
from gaze_detection.inference.webcam.postproc import extract_eye_coords
from gaze_detection.inference.webcam.draw     import draw_bboxes
from utils                                   import init_logging, set_seed, configure_device

def main(
    model_path: str,
    conf_thres: float,
    index: int,
    width: int,
    height: int,
):
    # 1) 로깅/시드/디바이스
    init_logging()
    set_seed(42)
    configure_device(gpu_id=0)

    # 2) 모델 & 카메라 초기화
    model = load_model(model_path=model_path, conf_thres=conf_thres)
    cap   = init_video_capture(index=index, width=width, height=height)

    logging.info("▶️ 시작: 'q' 키를 눌러 종료")

    # 3) 메인 루프: 프레임 읽고 => 추론 => 후처리 => 그리기 => 화면 표시
    while True:
        ret, frame = cap.read()
        if not ret:
            logging.error("프레임을 읽을 수 없습니다.")
            break

        # inference + 후처리 + 시각화
        results = model.predict(source=frame, verbose=False)
        coords  = extract_eye_coords(results)
        draw_bboxes(frame, results)

        cv2.imshow("Webcam Eye Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 4) 자원 해제
    cap.release()
    cv2.destroyAllWindows()
