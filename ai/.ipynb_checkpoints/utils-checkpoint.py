# utils.py

import os
import logging
import random
from typing import Union

import numpy as np

try:
    import torch
except ImportError:
    torch = None  # GPU/torch 관련 기능은 torch 설치 시에만 동작

LOGGER = logging.getLogger(__name__)


def configure_device(gpu_id: Union[int, str]) -> None:
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
        os.environ['CUDA_VISIBLE_DEVICES'] = str(gpu_index)
        LOGGER.info(f'Using GPU device CUDA_VISIBLE_DEVICES={gpu_index}')
    else:
        # 음수일 경우 GPU 비활성화(모두 사용 금지 → CPU 모드)
        os.environ['CUDA_VISIBLE_DEVICES'] = ''
        LOGGER.info('CUDA_VISIBLE_DEVICES unset, running on CPU')


def init_logging(level: int = logging.INFO) -> None:
    """
    루트 로거를 설정합니다. 기본 포맷과 레벨을 지정합니다.

    Args:
        level (int): 로깅 레벨 (default: INFO)
    """
    fmt = '%(asctime)s | %(name)s | %(levelname)s | %(message)s'
    datefmt = '%Y-%m-%d %H:%M:%S'
    logging.basicConfig(level=level, format=fmt, datefmt=datefmt)
    # ultralytics 등 외부 라이브러리 로거 레벨을 WARN 이상으로 올려서 출력을 줄일 수도 있습니다.
    logging.getLogger('ultralytics').setLevel(logging.WARNING)
    LOGGER.debug('Logging initialized')


def set_seed(seed: int) -> None:
    """
    랜덤 시드를 고정하여 실험 재현성을 확보합니다.

    Args:
        seed (int): 시드 값
    """
    random.seed(seed)
    np.random.seed(seed)
    LOGGER.info(f'Python random seed set to {seed}')
    if torch is not None:
        torch.manual_seed(seed)
        LOGGER.info(f'Torch CPU seed set to {seed}')
        if torch.cuda.is_available():
            torch.cuda.manual_seed(seed)
            torch.cuda.manual_seed_all(seed)
            # torch.backends.cudnn 재현성 옵션
            torch.backends.cudnn.deterministic = True
            torch.backends.cudnn.benchmark = False
            LOGGER.info('Torch CUDA seeds set and cuDNN deterministic mode enabled')
    else:
        LOGGER.warning('torch가 설치되어 있지 않아 GPU 시드 고정을 건너뜁니다.')
