# gaze_detection/data/download.py
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Final, Mapping, Tuple

from roboflow import Roboflow

__all__: Final = ['download_dataset']


def download_dataset(
    cfg: Mapping[str, Any],
    *,
    dest_dir: Path,
) -> Tuple[Path, Path]:
    """
    데이터셋을 지정한 경로에 다운로드하거나 이미 존재하면 재사용합니다.

    Args:
        cfg (Mapping[str, Any]): Roboflow 설정 (api_key, workspace, project_name, version, data_fmt).
        dest_dir (Path): 데이터셋 저장 또는 재사용할 로컬 디렉터리 경로.

    Returns:
        Tuple[Path, Path]: 데이터셋 루트 경로와 `data.yaml` 파일 경로를 튜플로 반환합니다.
    """
    yaml_path = dest_dir / 'data.yaml'
    if yaml_path.exists():
        logging.info('🗂️ 기존 데이터셋 재사용: %s', dest_dir)
        return dest_dir, yaml_path

    rf = Roboflow(api_key=str(cfg['api_key']))
    ver = (
        rf.workspace(str(cfg['workspace']))
        .project(str(cfg['project_name']))
        .version(int(cfg['version']))
    )

    logging.info('📥 %s v%d 다운로드 중 …', cfg['project_name'], cfg['version'])
    dataset = ver.download(str(cfg['data_fmt']), location=str(dest_dir))

    root = Path(dataset.location)
    return root, root / 'data.yaml'
