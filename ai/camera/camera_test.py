# import cv2

# def main():
#     # 0번 카메라(기본 웹캠) 열기
#     cap = cv2.VideoCapture(0)
#     if not cap.isOpened():
#         print("카메라를 열 수 없습니다.")
#         return

#     # 프레임 읽어오기 및 출력 루프
#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             print("프레임을 받아올 수 없습니다. 종료합니다.")
#             break

#         # (옵션) 거울처럼 보이게 좌우 반전
#         frame = cv2.flip(frame, 1)

#         # 창에 영상 표시
#         cv2.imshow('Webcam', frame)

#         # 키 입력 대기: 1ms마다 체크, 'q' 키를 누르면 루프 종료
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     # 자원 해제
#     cap.release()
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()
import cv2

def main():
    # (1) 디바이스 번호 + 백엔드 강제 지정
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    print("Camera opened:", cap.isOpened())
    if not cap.isOpened():
        print("→ 장치를 열지 못했습니다. 장치 번호나 권한을 확인하세요.")
        return

    while True:
        ret, frame = cap.read()
        # (2) 프레임 리드 체크
        if not ret:
            print("→ 프레임을 받아오지 못했습니다. 카메라 상태를 점검하세요.")
            break

        cv2.imshow('Webcam', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
