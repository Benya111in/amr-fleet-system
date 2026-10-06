"""
Pytest 설정 — rl/eval 를 import 경로에 넣고, 이론편 자체 계산 스크립트(calc*.py) 교차 확인 도구를 준다.

calc*.py 는 모듈 수준에서 긴 계산을 바로 돌리므로 import 하지 않는다. 대신 AST 에서 지정한 함수 정의만
꺼내 실행한다 — 같은 입력에서 우리 구현과 리서치 스크립트가 같은 값을 내는지 본다.
스크립트가 없는 환경(다른 clone)에서는 그 시험만 건너뛴다.
"""

import ast
import math
import os
import sys

import numpy as np
import pytest
from scipy import optimize, special, stats

HERE = os.path.dirname(os.path.abspath(__file__))
EVAL = os.path.dirname(HERE)
if EVAL not in sys.path:
    sys.path.insert(0, EVAL)

CALC_DIR = os.environ.get('RL_THEORY_CALC_DIR',
                          '/home/CAPYI_AI/research/artifacts/rl_theory_stats')


def load_calc(name, functions):
    """리서치 calc 스크립트에서 함수 정의만 꺼내 실행한 이름공간을 돌려준다 (없으면 skip)."""
    path = os.path.join(CALC_DIR, name)
    if not os.path.exists(path):
        pytest.skip(f'{path} 없음 — 리서치 스크립트 교차 확인 생략')
    with open(path, encoding='utf-8') as f:
        tree = ast.parse(f.read())
    keep = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in functions]
    missing = set(functions) - {n.name for n in keep}
    if missing:
        raise AssertionError(f'{name} 에 함수가 없다: {sorted(missing)}')
    mod = ast.Module(body=keep, type_ignores=[])
    ns = {'np': np, 'math': math, 'stats': stats, 'optimize': optimize, 'special': special,
          'z': stats.norm.ppf, 'EPS': 1e-12}
    exec(compile(mod, path, 'exec'), ns)  # noqa: S102 — 신뢰하는 로컬 리서치 스크립트
    return ns
