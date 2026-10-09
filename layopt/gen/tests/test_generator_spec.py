"""
월드 생성기 배치 사양(--spec) 회귀: 사양이 없거나 기준 배치 사양이면 커밋된 warehouse.sdf 와 바이트 단위로 같다.

표준 라이브러리만 쓴다 (호스트 python3 로도 돈다).
"""

import importlib.util
import json
import subprocess
import sys

from layopt.gen import levels as L
from layopt.gen.common import GEN_PATH

WORLD = GEN_PATH.parent / 'warehouse.sdf'


def fresh_gen():
    spec = importlib.util.spec_from_file_location('gen_fresh', GEN_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_no_spec_is_identical():
    sdf, info = fresh_gen().build_sdf(None)
    assert sdf == WORLD.read_text()
    assert info['racks'] == 25 and info['signs'] == 25


def test_baseline_levels_spec_is_identical():
    res = L.resolve(L.parse_spec_id(L.BASELINE_ID))
    gen = fresh_gen()
    assert res['gen_spec'] == gen.default_layout()
    sdf, _ = gen.build_sdf(json.loads(json.dumps(res['gen_spec'])))
    assert sdf == WORLD.read_text()


def test_repeated_builds_reset_random_state():
    gen = fresh_gen()
    variant = L.resolve(L.parse_spec_id('v12-v21-v32-v42-v51'))['gen_spec']
    a, _ = gen.build_sdf(variant)
    gen.build_sdf(None)
    b, _ = gen.build_sdf(variant)
    assert a == b
    base, _ = gen.build_sdf(None)
    assert base == WORLD.read_text() and a != base


def test_cli_spec_file(tmp_path):
    spec = tmp_path / 'spec.json'
    spec.write_text(json.dumps(fresh_gen().default_layout()))
    out = tmp_path / 'w.sdf'
    r = subprocess.run([sys.executable, str(GEN_PATH), str(out), '--spec', str(spec)],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert out.read_bytes() == WORLD.read_bytes()


def test_unknown_spec_key_rejected():
    gen = fresh_gen()
    try:
        gen.set_layout({'rackz': []})
    except ValueError as exc:
        assert 'rackz' in str(exc)
    else:
        raise AssertionError('모르는 키를 받아들였다')


def test_variant_changes_racks_and_markers():
    gen = fresh_gen()
    res = L.resolve(L.parse_spec_id('v10-v20-v30-v43-v52'))
    sdf, _ = gen.build_sdf(res['gen_spec'])
    assert '<name>rack_A8</name>' in sdf and '<name>rack_C7</name>' not in sdf
    lm = {**gen.pillar_marker_poses(), **gen.rack_marker_poses()}
    assert sorted(lm) == list(range(10, 84))
    # 기둥 8 (25, -6) → (25, -9.5): 그 기둥 마커 id 38..41 이 따라간다
    assert abs(lm[38][1] - (-9.5)) < 1e-9
