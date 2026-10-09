"""
유효성 가드 시험 (numpy · scipy · yaml — 컨테이너): 무효여야 할 배치가 무효로, 구역 간격 규칙이 지켜지는지.

변이 검사(COORDINATION §4) 대상: validate.check 의 lane · slot · zones 가드, zones 의 간격 병합, levels 의 구조물 자리.
"""

import copy

from layopt.gen import build_bundle as B, levels as L, raster as R, validate as V, zones as Z
from layopt.gen.common import load_generator


def _check(res, tmp_path):
    gen = load_generator()
    sdf, _ = gen.build_sdf(res['gen_spec'])
    (tmp_path / 'w.sdf').write_text(sdf)
    raster, unknown, _ = R.rasterize_world(tmp_path / 'w.sdf')
    obstacles = L._static_obstacles(res['gen_spec'])
    tz, notes = Z.derive(res, obstacles)
    from layopt.gen import configs
    text = configs.traffic_zones_yaml(tz, res['spec_id'], notes)
    return V.check(res, gen, raster, unknown, tz, obstacles, text)


def test_known_invalid_lane(tmp_path):
    v = B.build(L.parse_spec_id('v14-v20-v31-v40-v50'), tmp_path / 'a', 't', False, seal=False)
    assert not v['valid'] and v['reason_kinds'] == ['lane']
    assert any('forklift_main' in r for r in v['reasons'])


def test_known_invalid_structure(tmp_path):
    v = B.build(L.parse_spec_id('v10-v21-v31-v40-v51'), tmp_path / 'b', 't', False, seal=False)
    assert not v['valid'] and v['reason_kinds'] == ['resolve']
    assert not (tmp_path / 'b' / 'world.sdf').exists()


def test_slot_into_rack_is_invalid(tmp_path):
    res = copy.deepcopy(L.resolve(L.parse_spec_id(L.BASELINE_ID)))
    res['slots'][0].update(x=-18.0, y=8.2)          # 랙 A1 남면(8.5)에서 0.3 m
    ok, reasons, _ = _check(res, tmp_path)
    assert not ok and any(r.startswith('slot:slot amr_01') for r in reasons)


def test_slot_on_walker_lane_is_invalid(tmp_path):
    res = copy.deepcopy(L.resolve(L.parse_spec_id(L.BASELINE_ID)))
    res['slots'][4].update(x=22.0, y=-9.0)          # worker_straight_fast 끝 (22, -10) 에서 1.0 m
    ok, reasons, _ = _check(res, tmp_path)
    assert not ok and any(r.startswith('lane:slot amr_05') for r in reasons)


def test_pitch_mix_zones_merged_without_conflicts():
    res = L.resolve(L.parse_spec_id('v10-v20-v30-v42-v50'))
    tz, _ = Z.derive(res, L._static_obstacles(res['gen_spec']))
    widths = [max(p[0] for p in z['polygon']) - min(p[0] for p in z['polygon'])
              for z in tz['zones'] if z['id'].startswith('x_')]
    assert max(widths) > 2.0 + 1e-9
    from layopt.gen import configs
    reasons, info = V.zone_check(configs.traffic_zones_yaml(tz, res['spec_id']))
    assert reasons == [] and info['spacing_conflicts'] == 0


def test_spacing_conflict_detected():
    res = L.resolve(L.parse_spec_id(L.BASELINE_ID))
    tz, _ = Z.derive(res, L._static_obstacles(res['gen_spec']))
    tz = copy.deepcopy(tz)
    z = next(z for z in tz['zones'] if z['id'] == 'x_ab_2')
    for p in z['polygon']:                          # x_ab_1 (-16..-14) 과 간격 2.0 m 로
        p[0] -= 2.0
    from layopt.gen import configs
    reasons, _ = V.zone_check(configs.traffic_zones_yaml(tz, res['spec_id']))
    assert any(r.startswith('zones:spacing x_ab_1-x_ab_2') for r in reasons)


def test_structure_on_rack_is_overlap(tmp_path):
    res = copy.deepcopy(L.resolve(L.parse_spec_id(L.BASELINE_ID)))
    res['gen_spec']['structures'] = [{'id': 'S9', 'kind': 'pallet_stack', 'x': -12.0, 'y': 9.9,
                                      'yaw': 0.0, 'l': 1.2, 'd': 1.0, 'h': 1.5}]
    ok, reasons, _ = _check(res, tmp_path)
    assert not ok and any(r.startswith('overlap:rack_A2~structure_S9') for r in reasons)


def test_merge_rule_on_synthetic_rows():
    """같은 통로 이웃 구역 간격이 1.5 < g < 2.36 m 면 합친다 (현행 수준에서는 생기지 않는 안전장치)."""
    def spans(rows):
        return [(min(p[0] for p in z['polygon']), max(p[0] for p in z['polygon']))
                for z in Z.intersection_zones(rows)]
    rows = {'A': {'y': 9.0, 'xs': [-18.0, -12.0, -6.0]},          # 틈 -15, -9
            'B': {'y': 3.0, 'xs': [-18.0, -12.0, -6.0]}}
    assert spans(rows) == [(-16.0, -14.0), (-10.0, -8.0)]          # 간격 4 m — 그대로
    rows['B']['xs'] = [-18.0, -8.0, -2.0]                           # 틈 -13 (범위 안), -5 (밖)
    # 구역 (-16,-14) (-14,-12) (-10,-8): 간격 0 (사슬) · 2.0 (충돌 → 병합)
    assert spans(rows) == [(-16.0, -14.0), (-14.0, -8.0)]


def test_queue_points_need_open_approach():
    """좁은 통로 옆면·도크 화물 너머·랙이 막은 T 자 교차로 쪽 대기 자세는 만들지 않는다."""
    from layopt.gen import dwell
    cases = (('v10-v20-v30-v40-v50', ('queue:narrow_aisle:w', 'queue:narrow_aisle:e',
                                      'queue:dock_2_approach:s', 'queue:dock_b_approach:s')),
             ('v10-v20-v30-v41-v50', ('queue:x_ab_2:n', 'queue:x_bc_2:s')))
    for sid, banned in cases:
        res = L.resolve(L.parse_spec_id(sid))
        obstacles = L._static_obstacles(res['gen_spec'])
        tz, _ = Z.derive(res, obstacles)
        ids = {p['id'] for p in dwell.dwell_points(res, tz, obstacles)}
        assert not ids & set(banned), sorted(ids & set(banned))
        assert {'queue:narrow_aisle:n', 'queue:narrow_aisle:s'} <= ids
