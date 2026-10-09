"""
묶음 생성 회귀 (numpy · scipy · yaml — 컨테이너).

기준 배치 묶음 = 저장소 원본 (월드 바이트, 설정 해석값), 변형 묶음 형식.
"""

import json

import pytest
import yaml

from layopt.gen import build_bundle as B, compare_baseline as C, levels as L


@pytest.fixture(scope='module')
def baseline(tmp_path_factory):
    out = tmp_path_factory.mktemp('b') / L.BASELINE_ID
    v = B.build(L.parse_spec_id(L.BASELINE_ID), out, 'test', False, seal=False)
    return out, v


def test_baseline_valid_and_identical(baseline):
    out, v = baseline
    assert v['valid'], v['reasons']
    rep = C.compare(out)
    assert rep['ok'], json.dumps(rep, ensure_ascii=False)


def test_contract_files(baseline):
    out, _ = baseline
    for name in ('spec.json', 'layout.json', 'world.sdf', 'map.pgm', 'map.yaml', 'valid.json',
                 'config/behavior.yaml', 'config/fleet_spawn.yaml', 'config/traffic_zones.yaml'):
        assert (out / name).is_file(), name
    lj = json.loads((out / 'layout.json').read_text())
    kinds = {p['kind'] for p in lj['dwell_points']}
    assert kinds == {'waiting', 'dock', 'charger', 'queue'}
    w1 = next(p for p in lj['dwell_points'] if p['id'] == 'wait:amr_01')
    assert w1['neighbors'][0] == [20.0, -16.0, 1.5708]
    assert len(lj['markers']) == 4 + 3 + 74
    m = yaml.safe_load((out / 'map.yaml').read_text())
    assert m['resolution'] == 0.05 and m['origin'][:2] == [-30.5, -20.5]


def test_refuses_existing(baseline):
    out, _ = baseline
    before = (out / 'spec.json').read_text()
    with pytest.raises(FileExistsError):
        B.build(L.parse_spec_id(L.BASELINE_ID), out, 'test', False, seal=False)
    assert (out / 'spec.json').read_text() == before
    assert not list(out.parent.glob(f'{out.name}.tmp-*')), '거부했는데 임시 디렉토리를 만들었다'


@pytest.mark.parametrize('sid', ['v12-v22-v31-v42-v52', 'v14-v21-v32-v41-v51',
                                 'v11-v20-v31-v43-v51', 'v10-v21-v31-v40-v51'])
def test_variant_bundle(tmp_path, sid):
    v = B.build(L.parse_spec_id(sid), tmp_path / sid, 'test', False, seal=False)
    if v['valid']:
        beh = yaml.safe_load((tmp_path / sid / 'config' / 'behavior.yaml').read_text())
        reloc = beh['/**/docking_server_node']['ros__parameters']['relocalization']
        assert len(reloc['landmark_poses']) == 3 * len(reloc['landmark_ids']) == 3 * 74
        fs = yaml.safe_load((tmp_path / sid / 'config' / 'fleet_spawn.yaml').read_text())
        assert [r['name'] for r in fs['robots']] == L.ROBOTS
    else:
        assert v['reasons'] and all(':' in r for r in v['reasons'])
