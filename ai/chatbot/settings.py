# chatbot/settings.py
import os
import yaml
from pathlib import Path

# 1) 이 파일이 있는 디렉터리를 기준으로
BASE_DIR = Path(__file__).parent

# 2) config.yaml 로딩
config_path = BASE_DIR / "config.yaml"
if not config_path.exists():
    raise FileNotFoundError(f"{config_path} not found")
with open(config_path, encoding="utf-8") as f:
    cfg = yaml.safe_load(f)

# 3) 환경변수 오버라이드 허용
OPENAI_API_KEY    = os.getenv("OPENAI_API_KEY", cfg["openai"]["api_key"])
OPENAI_API_BASE   = os.getenv("OPENAI_API_BASE", cfg["openai"]["api_base"])
ANTHROPIC_API_KEY = os.getenv("ANTHROPIC_API_KEY", cfg["anthropic"]["api_key"])
GOOGLE_CRED       = os.getenv("GOOGLE_APPLICATION_CREDENTIALS", cfg["google"]["credentials"])
GMS_MODEL         = cfg["model"]["name"]
TEMPERATURE       = cfg["model"]["temperature"]
MANUAL_PATH       = BASE_DIR / cfg["manual_path"]

# 4) 필수값 체크
for name, val in [
    ("OPENAI_API_KEY", OPENAI_API_KEY),
    ("OPENAI_API_BASE", OPENAI_API_BASE),
    ("MANUAL_PATH", MANUAL_PATH),
]:
    if not val:
        raise RuntimeError(f"Missing config: {name}")
