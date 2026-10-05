"""결과 CSV 계약 — 검증 규칙."""

from amr_rl_eval import schema as S
import pytest


def _row(**kw):
    r = {'schema_version': S.SCHEMA_VERSION, 'arm': 'C_hybrid', 'policy_id': 'C/s1/1M',
         'train_seed': '1', 'simulator': 'gazebo', 'scenario_set': 'gz_new600',
         'scenario_id': 'aisle_07', 'scenario_seed': '1234', 'difficulty': 'dynamic_high',
         'episode': '0', 'run_tag': 'R4C001', 'commit': 'deadbee', 'safety_node': 'on',
         'outcome': 'success', 'contacts': '0', 'time_s': '17.2'}
    r.update(kw)
    return r


def test_valid_minimal_row():
    row, errs = S.validate_row(_row())
    assert errs == [] and row['scenario_seed'] == 1234 and row['cte_mean_m'] is None


@pytest.mark.parametrize('kw,frag', [
    ({'arm': 'E_magic'}, 'arm'), ({'simulator': 'mujoco'}, 'simulator'),
    ({'outcome': 'crash'}, 'outcome'), ({'contacts': '2'}, 'success 인데'),
    ({'outcome': 'collision', 'contacts': '0'}, 'collision 인데'),
    ({'scenario_seed': ''}, '필수'), ({'episode': '1.5'}, '정수'),
    ({'schema_version': 'v0'}, 'schema_version'), ({'bogus': '1'}, '모르는 열'),
    ({'contacts': '-1', 'outcome': 'timeout'}, 'contacts < 0'),
])
def test_invalid_rows(kw, frag):
    _, errs = S.validate_row(_row(**kw))
    assert any(frag in e for e in errs), errs


def test_roundtrip_and_duplicate_key(tmp_path):
    rows = [S.validate_row(_row(episode=str(i)))[0] for i in range(3)]
    p = tmp_path / 'r.csv'
    S.write_csv(str(p), rows)
    back = S.read_csv(str(p))
    assert [r['episode'] for r in back] == [0, 1, 2]
    S.write_csv(str(p), rows + rows[:1])
    with pytest.raises(ValueError, match='중복'):
        S.read_csv(str(p))


def test_strict_reports_line_numbers(tmp_path):
    p = tmp_path / 'r.csv'
    S.write_csv(str(p), [S.validate_row(_row())[0]])
    txt = p.read_text().replace('C_hybrid', 'Z_bad')
    p.write_text(txt)
    with pytest.raises(ValueError, match=r'r\.csv:2'):
        S.read_csv(str(p))
    assert len(S.read_csv(str(p), strict=False)) == 1


def test_markdown_table_lists_every_column():
    md = S.markdown_table()
    for name in S.NAMES:
        assert f'`{name}`' in md
    # 칸 안에 '|' 가 있으면 마크다운 표가 깨진다 — 모든 줄의 칸 수가 같아야 한다
    assert {line.count('|') for line in md.splitlines()} == {5}


def test_protocol_table_matches_schema():
    import os
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(here, 'PROTOCOL.md'), encoding='utf-8') as f:
        assert S.markdown_table() in f.read(), 'PROTOCOL.md §5 표를 schema.markdown_table() 로 갱신'
