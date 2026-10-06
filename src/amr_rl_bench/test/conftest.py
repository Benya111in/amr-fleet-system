"""시험 공통: 패키지 소스 경로와 저장소 경로."""

import os
import sys

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.environ.get('REPO_DIR') or os.path.normpath(os.path.join(PKG, '..', '..'))
sys.path.insert(0, PKG)
