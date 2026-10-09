#!/usr/bin/env python3
"""
기준 배치 묶음 ↔ 저장소 원본 대조 (W1·W3 회귀, design §5).

    python3 -m layopt.gen.compare_baseline <묶음 디렉토리>      # JSON 보고, 차이가 있으면 종료 코드 1

- world.sdf  == src/amr_simulation/worlds/warehouse.sdf (바이트)
- config/behavior.yaml, config/fleet_spawn.yaml == 원본 (yaml 해석값이 같다 — 주석·줄 배치는 다를 수 있다)
- config/traffic_zones.yaml ≈ 원본 (구역 id·종류·같은 방향·꼭짓점, 포켓 id·x·y·yaw, 허용 1e-6)
numpy 불필요, yaml 필요.
"""

import filecmp
import json
import math
from pathlib import Path
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from layopt.gen.common import (BEHAVIOR_SRC, FLEET_SPAWN_SRC, GEN_PATH, ZONES_SRC)  # noqa: E402


def _diff(a, b, path='', tol=1e-6, out=None):
    """두 yaml 값의 차이 [(경로, a, b)] (수는 tol 안이면 같다)."""
    out = [] if out is None else out
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b), key=str):
            if k not in a or k not in b:
                out.append((f'{path}/{k}', a.get(k, '<없음>'), b.get(k, '<없음>')))
            else:
                _diff(a[k], b[k], f'{path}/{k}', tol, out)
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            out.append((f'{path}#len', len(a), len(b)))
        for i, (x, y) in enumerate(zip(a, b)):
            _diff(x, y, f'{path}[{i}]', tol, out)
    elif isinstance(a, (int, float)) and isinstance(b, (int, float)) \
            and not isinstance(a, bool) and not isinstance(b, bool):
        if not math.isclose(float(a), float(b), abs_tol=tol):
            out.append((path, a, b))
    elif a != b:
        out.append((path, a, b))
    return out


def compare(bundle):
    bundle = Path(bundle)
    rep = {'bundle': str(bundle)}
    rep['world_identical'] = filecmp.cmp(bundle / 'world.sdf', GEN_PATH.parent / 'warehouse.sdf',
                                         shallow=False)
    for name, src in (('behavior.yaml', BEHAVIOR_SRC), ('fleet_spawn.yaml', FLEET_SPAWN_SRC),
                      ('traffic_zones.yaml', ZONES_SRC)):
        a = yaml.safe_load((bundle / 'config' / name).read_text(encoding='utf-8'))
        b = yaml.safe_load(Path(src).read_text(encoding='utf-8'))
        d = _diff(a, b)
        rep[name] = {'n_diff': len(d), 'diff': [[p, str(x), str(y)] for p, x, y in d[:50]]}
    rep['ok'] = rep['world_identical'] and all(
        rep[n]['n_diff'] == 0 for n in ('behavior.yaml', 'fleet_spawn.yaml', 'traffic_zones.yaml'))
    return rep


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    rep = compare(argv[0])
    print(json.dumps(rep, ensure_ascii=False, indent=1))
    return 0 if rep['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
