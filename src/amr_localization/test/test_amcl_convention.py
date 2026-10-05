"""
amcl_map_adapter 의 반 셀 보정을 설치된 nav2_amcl 매크로로 확인한다 (식을 다시 쓰지 않고 헤더에서 읽음).

nav2_amcl (humble) map/map.hpp 의 MAP_GXWX / MAP_WXGX 매크로 문자열을 그대로 파이썬 식으로 옮겨 평가하고,
amcl_node.cpp convertMap 의 원점 이동 (origin + size/2 · scale; 헤더에 없어 소스 규약으로 둔다)을 더해,
보정 맵을 받은 AMCL 의 월드→셀 변환이 OccupancyGrid 규약 floor((x − o)/s) 와 같고 셀→월드가 셀 중심인지 본다.
nav2_amcl 이 매크로를 바꾸면(반올림 규약 변경) 이 테스트가 실패해 보정을 다시 판단하게 한다.
"""

import math
from pathlib import Path
import re

from amr_localization.amcl_map_adapter import half_cell_offset
import numpy as np
import pytest


def header_path():
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = Path(get_package_prefix('nav2_amcl'))
    except Exception:   # noqa: B902 — nav2_amcl 이 없는 환경
        prefix = Path('/opt/ros/humble')
    path = prefix / 'include' / 'nav2_amcl' / 'map' / 'map.hpp'
    return path if path.is_file() else None


def macro(text, name):
    """#define NAME(map, v) (expr) → 파이썬 람다 (map->필드는 dict 조회, C 정수 나눗셈 size/2 는 //)."""
    m = re.search(rf'#define {name}\(map, (\w)\) (.+)', text)
    assert m, name
    var, expr = m.group(1), m.group(2)
    expr = re.sub(r'map->(\w+)', r'M["\1"]', expr)
    expr = expr.replace('M["size_x"] / 2', 'M["size_x"] // 2').replace(
        'M["size_y"] / 2', 'M["size_y"] // 2')
    return eval(f'lambda M, {var}: {expr}', {'floor': math.floor})   # noqa: S307 — 설치 헤더 식


@pytest.mark.skipif(header_path() is None, reason='nav2_amcl headers not installed')
def test_adapter_matches_installed_nav2_amcl_macros():
    text = header_path().read_text()
    gxwx = macro(text, 'MAP_GXWX')
    wxgx = macro(text, 'MAP_WXGX')
    s, o, n = 0.05, -30.1783, 1207
    rng = np.random.default_rng(0)
    xs = rng.uniform(o, o + n * s, 2000)

    def amcl_map(origin):
        # convertMap: origin_x = msg.origin.x + (size_x / 2) · scale
        return {'origin_x': origin + (n // 2) * s, 'scale': s, 'size_x': n}

    raw = amcl_map(o)
    fixed = amcl_map(o + half_cell_offset(s)[0])
    ros_cells = np.floor((xs - o) / s)
    raw_cells = np.array([gxwx(raw, x) for x in xs])
    fixed_cells = np.array([gxwx(fixed, x) for x in xs])
    # 보정 없이: 셀 경계에서 반 셀 앞당겨 반올림 → 약 절반의 점이 다음 셀로 (지도 전체 −s/2 이동과 같음)
    assert 0.3 < np.mean(raw_cells != ros_cells) < 0.7
    # 보정 후: OccupancyGrid 규약과 일치, 셀 → 월드는 셀 중심
    assert np.array_equal(fixed_cells, ros_cells)
    for i in (0, 17, n // 2, n - 1):
        assert wxgx(fixed, i) == pytest.approx(o + (i + 0.5) * s)
