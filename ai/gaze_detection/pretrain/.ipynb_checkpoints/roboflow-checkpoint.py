"""
Pre-train용 데이터셋(Roboflow) 다운로드 헬퍼.

예시:
    >>> from gaze_detection.pretrain.roboflow import download_rf_dataset
    >>> yaml_path = download_rf_dataset(
    ...     api_key='XXXX',
    ...     workspace='iseeds-workspace',
    ...     project_name='iseeds',
    ...     version=8,
    ...     data_fmt='yolov11',
    ... )
    >>> print(yaml_path)  # datasets/iseeds‑8/data.yaml
"""

from __future__ import annotations

import logging
from pathlib import Path
# gaze_detection/pretrain/roboflow.py

from typing import Final

from roboflow import Roboflow

__all__: Final = ['download_rf_dataset']


def download_rf_dataset(
    *,
    api_key: str,
    workspace: str,
    project_name: str,
    version: int,
    data_fmt: str,
) -> Path:
    """
    Roboflow에서 데이터셋을 다운로드하고 data.yaml 경로를 반환합니다.

    Args:
        api_key (str): Roboflow API Key
        workspace (str): Roboflow 워크스페이스 이름
        project_name (str): 프로젝트 이름
        version (int): 버전 번호
        data_fmt (str): 다운로드 포맷 (예: 'yolov11')

    Returns:
        Path: 로컬에 저장된 data.yaml 파일 경로
    """
    logging.info('▶️  [PRETRAIN] 데이터셋 다운로드 시작')
    rf = Roboflow(api_key=api_key)

    logging.info('워크스페이스 로드: %s', workspace)
    ws = rf.workspace(workspace)

    logging.info('프로젝트 로드: %s (v%d)', project_name, version)
    proj = ws.project(project_name)
    ver = proj.version(version)

    logging.info('데이터셋 다운로드 (%s 포맷)…', data_fmt)
    dataset = ver.download(data_fmt)
    dataset_path = Path(dataset.location)

    yaml_path = dataset_path / 'data.yaml'
    logging.info('데이터셋 완료 → %s', yaml_path)

    return yaml_path
