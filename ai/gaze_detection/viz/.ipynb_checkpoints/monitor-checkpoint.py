# gaze_detection/viz/monitor.py

from __future__ import annotations

from typing import Final, Dict

from torch.utils.tensorboard.writer import SummaryWriter

__all__: Final = ['init_tensorboard', 'log_metrics']


def init_tensorboard(log_dir: str) -> SummaryWriter:
    """
    SummaryWriter 객체를 생성하여 반환합니다.

    Args:
        log_dir (str): TensorBoard 로그를 기록할 디렉터리 경로

    Returns:
        SummaryWriter: 초기화된 writer 객체
    """
    return SummaryWriter(log_dir)


def log_metrics(writer: SummaryWriter, metrics: Dict[str, float], step: int) -> None:
    """
    주어진 지표들을 TensorBoard에 기록합니다.

    Args:
        writer (SummaryWriter): SummaryWriter 객체
        metrics (dict): 기록할 스칼라 딕셔너리 (예: {'loss': 0.123})
        step (int): 기록할 스텝 인덱스 (에폭/배치 등)
    """
    for key, value in metrics.items():
        writer.add_scalar(key, value, step)
