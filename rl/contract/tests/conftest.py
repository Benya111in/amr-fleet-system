"""Pytest 설정 — rl/contract 를 import 경로에 넣는다."""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CONTRACT = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(CONTRACT))
for p in (CONTRACT, os.path.join(CONTRACT, 'golden'), os.path.join(CONTRACT, 'onnx')):
    if p not in sys.path:
        sys.path.insert(0, p)
