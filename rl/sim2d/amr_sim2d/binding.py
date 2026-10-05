"""dwa_binding/amr_core_py 모듈 찾기 (없으면 build.sh 로 빌드)."""

import glob
import importlib
import os
import subprocess
import sys

from ._contract import SIM2D_DIR

BUILD_DIR = os.path.join(SIM2D_DIR, 'dwa_binding', 'build')
_MOD = None


def core_module(build=True):
    """amr_core_py 를 import 해 돌려준다."""
    global _MOD
    if _MOD is not None:
        return _MOD
    if not glob.glob(os.path.join(BUILD_DIR, 'amr_core_py*.so')) and build:
        subprocess.run(['bash', os.path.join(SIM2D_DIR, 'dwa_binding', 'build.sh')], check=True)
    if BUILD_DIR not in sys.path:
        sys.path.insert(0, BUILD_DIR)
    _MOD = importlib.import_module('amr_core_py')
    return _MOD
