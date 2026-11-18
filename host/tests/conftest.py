from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VECTORS_PATH = REPO_ROOT / "test" / "vectors" / "protocol_vectors.json"
DATA_DIR = Path(__file__).resolve().parent / "data"


@pytest.fixture(scope="session")
def vectors() -> dict[str, Any]:
    data: dict[str, Any] = json.loads(VECTORS_PATH.read_text(encoding="utf-8"))
    return data


@pytest.fixture
def data_dir() -> Path:
    return DATA_DIR
