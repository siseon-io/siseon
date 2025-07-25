# draw.py

import cv2

def draw_bboxes(frame, results) -> None:
    """
    검출된 박스를 원본 프레임에 그리고
    클래스 이름을 찍어줍니다.
    """
    for box in results[0].boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
        cls_id = int(box.cls[0].cpu().numpy())
        label  = results[0].names[cls_id]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,255), 2)
        cv2.putText(
            frame,
            label,
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0,255,255),
            1
        )
