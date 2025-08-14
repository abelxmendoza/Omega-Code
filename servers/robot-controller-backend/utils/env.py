# utils/env.py
import os
from pathlib import Path

try:
    # optional: load .env if python-dotenv is installed
    from dotenv import load_dotenv  # type: ignore
    _env = Path(__file__).resolve().parents[1] / ".env"
    if _env.exists():
        load_dotenv(_env)
except Exception:
    # silently ignore if python-dotenv not installed; os.environ still works
    pass

def env_str(key: str, default: str) -> str:
    return os.getenv(key, default)

def env_int(key: str, default: int) -> int:
    try:
        return int(os.getenv(key, str(default)))
    except Exception:
        return default
