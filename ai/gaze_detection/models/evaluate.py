import logging

def evaluate_model(model, cfg: dict, yaml_path, split: str = "val"):
    """
    모델을 주어진 split('val' or 'test')에 대해 평가하고
    mAP@0.5 와 mAP@0.5:0.95 를 함께 로그로 남깁니다.
    Returns:
        metrics (ultralytics.utils.metrics.DetMetrics): 평가 결과 객체
    """
    logging.info(f"Evaluating split={split}")
    metrics = model.val(data=str(yaml_path), split=split)

    mp50    = metrics.box.map50   # IoU=0.5
    mp5095  = metrics.box.map     # IoU=0.5:0.95
    logging.info(f"{split} mAP@0.5: {mp50:.4f}, mAP@0.5:0.95: {mp5095:.4f}")

    return metrics
