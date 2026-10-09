"""layopt/gen 시험 공용: 저장소 루트를 import 경로에 넣는다 (python3 -m pytest layopt/gen/tests)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
