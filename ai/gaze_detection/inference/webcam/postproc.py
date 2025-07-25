# postproc.py

def extract_eye_coords(results) -> dict:
    """
    YOLO 결과에서 lefteye/righteye 클래스의 중심 좌표 추출
    """
    coords = {
        "lefteye_x": None, "lefteye_y": None,
        "righteye_x": None, "righteye_y": None
    }
    for box in results[0].boxes:
        cls_id = int(box.cls[0].cpu().numpy())
        name   = results[0].names[cls_id].lower()
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        cx, cy = float((x1 + x2) / 2), float((y1 + y2) / 2)
        if name == "lefteye":
            coords["lefteye_x"], coords["lefteye_y"] = cx, cy
        elif name == "righteye":
            coords["righteye_x"], coords["righteye_y"] = cx, cy
    return coords