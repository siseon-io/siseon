# chatbot/settings.py
from __future__ import annotations

import os
import base64
from pathlib import Path
from dotenv import load_dotenv


# ──────────────────────────────────────────────────────────────────────────────
# 기본 설정: 컨테이너/호스트 환경변수를 우선 사용, .env는 비어있을 때만 채움
# ──────────────────────────────────────────────────────────────────────────────
BASE_DIR = Path(__file__).parent
load_dotenv(BASE_DIR / ".env", override=False)


# ──────────────────────────────────────────────────────────────────────────────
# 유틸
# ──────────────────────────────────────────────────────────────────────────────
def _get_required(name: str) -> str:
    v = os.getenv(name)
    if not v:
        raise RuntimeError(f"Missing required environment variable: {name}")
    return v

def _get_int(name: str, default: int | None = None) -> int:
    v = os.getenv(name)
    if v is None:
        if default is None:
            raise RuntimeError(f"Missing required integer environment variable: {name}")
        return default
    try:
        return int(v)
    except ValueError as e:
        raise RuntimeError(f"Invalid integer for {name}: {v}") from e


# ──────────────────────────────────────────────────────────────────────────────
# OpenAI / LLM
# ──────────────────────────────────────────────────────────────────────────────
OPENAI_API_KEY  = _get_required("OPENAI_API_KEY")
OPENAI_API_BASE = _get_required("OPENAI_API_BASE")
GMS_MODEL       = os.getenv("GMS_MODEL", "gpt-4.1")
TEMPERATURE     = float(os.getenv("TEMPERATURE", "0.0"))


# ──────────────────────────────────────────────────────────────────────────────
# Manual
# ──────────────────────────────────────────────────────────────────────────────
MANUAL_PATH = os.getenv("MANUAL_PATH", "/app/manuals/siseon_manual.pdf")


# ──────────────────────────────────────────────────────────────────────────────
# Database
# ──────────────────────────────────────────────────────────────────────────────
DB_HOST     = _get_required("DB_HOST")
DB_PORT     = _get_int("DB_PORT", 3306)
DB_USER     = _get_required("DB_USER")
DB_PASSWORD = _get_required("DB_PASSWORD")
DB_NAME     = _get_required("DB_NAME")


# ──────────────────────────────────────────────────────────────────────────────
# Spring Base (선택)
# ──────────────────────────────────────────────────────────────────────────────
SPRING_BASE_URL = os.getenv("SPRING_BASE_URL")


# ──────────────────────────────────────────────────────────────────────────────
# JWT
#  - Spring은 HS256 + base64url 인코딩된 secret을 "디코딩"해서 사용
#  - FastAPI도 동일하게 base64url 디코딩한 바이트 키로 검증해야 함
# ──────────────────────────────────────────────────────────────────────────────
# ※ 기존 ALGORITHM 변수명 사용하던 코드 호환 위해 여전히 읽긴 하되,
#   새 변수(JWT_ALGORITHM)를 우선한다.
JWT_ALGORITHM = os.getenv("JWT_ALGORITHM")
JWT_ISSUER    = os.getenv("JWT_ISSUER")

# Spring과 공유되는 base64url secret 문자열 (그대로 전달됨)
JWT_SECRET_KEY_B64URL = _get_required("JWT_SECRET_KEY")

# FastAPI 검증용: base64url → bytes
try:
    JWT_SECRET_KEY_BYTES = base64.urlsafe_b64decode(JWT_SECRET_KEY_B64URL)
    if not JWT_SECRET_KEY_BYTES:
        raise ValueError("decoded key is empty")
except Exception as e:
    raise RuntimeError(f"Failed to base64url-decode JWT_SECRET_KEY: {e}") from e