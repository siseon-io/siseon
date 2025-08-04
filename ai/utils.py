import os
import logging
import random
import numpy as np

LOGGER = logging.getLogger(__name__)


def configure_device(gpu_id: int | str) -> None:
    """
    GPU 사용 디바이스를 환경변수로 설정합니다.

    Args:
        gpu_id (int | str): 사용할 GPU ID (정수 또는 문자열)

    Raises:
        ValueError: gpu_id가 올바른 정수로 변환되지 않을 경우
    """
    try:
        gpu_index = int(gpu_id)
    except (TypeError, ValueError):
        raise ValueError(f'Invalid GPU ID: {gpu_id!r}. gpu_id는 정수 또는 정수 형태의 문자열이어야 합니다.')

    if gpu_index >= 0:
        os.environ['CUDA_VISIBLE_DEVICES']
