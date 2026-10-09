"""수준 표 · spec_id · 해석 (표준 라이브러리만)."""

import collections

import pytest

from layopt.gen import levels as L
from layopt.gen.common import load_generator


def test_540_unique_ids_roundtrip():
    ids = L.all_spec_ids()
    assert len(ids) == 540 == len(set(ids))
    assert ids[0] == L.BASELINE_ID
    for s in ids[::37]:
        assert L.spec_id(L.parse_spec_id(s)) == s


@pytest.mark.parametrize('bad', ['v15-v20-v30-v40-v50', 'v10-v23-v30-v40-v50', 'x',
                                 'v1-v2-v3-v4-v5'])
def test_bad_ids(bad):
    with pytest.raises(ValueError):
        L.parse_spec_id(bad)


def test_baseline_walk_graph_matches_generator():
    gen = load_generator()
    rows = L.rows_from_racks(gen.default_layout()['racks'])
    edges, _nodes = L.walk_graph(rows)
    want = [((float(a[0]), float(a[1])), (float(b[0]), float(b[1])))
            for a, b in gen.default_walk_edges()]
    assert edges == want


def test_every_variant_has_21_main_racks_and_resolves_or_says_why():
    kinds = collections.Counter()
    for s in L.all_spec_ids():
        try:
            res = L.resolve(L.parse_spec_id(s))
        except L.LayoutError as exc:
            kinds[str(exc).split(':')[0]] += 1
            continue
        assert len(res['gen_spec']['racks']) == 21
        assert len(res['slots']) == 5 and len(res['chargers']) == 3
        assert sorted(c['marker_id'] for c in res['chargers']) == [4, 5, 6]
    assert set(kinds) <= {'structure', 'chargers', 'lane'}


def test_levels_counts():
    assert [len(L.LEVELS[k]) for k in ('V1', 'V2', 'V3', 'V4', 'V5')] == [5, 3, 3, 4, 3]


def test_current_waiting_slots():
    res = L.resolve(L.parse_spec_id(L.BASELINE_ID))
    assert [(s['x'], s['y']) for s in res['slots']] == [(x, -16.0) for x in (18, 20, 22, 24, 26)]
