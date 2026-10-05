"""
결과 CSV 스키마 — 긴 형식 (한 행 = 한 실행·한 지표·한 통계).

한 bag 을 여러 행(slam_toolbox, RTAB-Map 2D/3D, LIO-SAM …)과 요인 수준으로 재생한 결과를 같은 파일에 쌓고,
짝지은 검정(같은 bag·같은 재생 번호)과 Holm 보정은 이 파일을 읽어 따로 한다 (evaluation.md §5).
실패한 실행도 행을 남긴다 (failed=1, value 는 NaN 가능) — 평균에서 빼지 않기 위해서다.
표본이 없는 통계(예: 정지 구간이 없는 궤적의 정지 RMSE)는 n = 0, value = NaN 으로 남긴다.
"""

import csv
import math
import re
from typing import Dict, Iterable, List, Optional

SCHEMA_VERSION = 1

# (열 이름, 형, 필수 여부, 설명)
COLUMNS = (
    ('schema', int, True, '스키마 판 (SCHEMA_VERSION)'),
    ('run_id', str, True, '재생 실행 하나의 고유 id'),
    ('scenario', str, True, 'S1–S7 | MAP(매핑 경로) | SYN(합성 시험)'),
    ('bag_id', str, True, '입력 bag id (같은 bag = 짝)'),
    ('seed', str, False, '월드·동적 장애물 씨드'),
    ('replay', int, True, '같은 bag·같은 구성의 재생 번호 (0 부터)'),
    ('row', str, True, '비교 행: slam_toolbox | rtabmap_2d | rtabmap_3d | liosam | … (자유 문자열)'),
    ('stage', str, True, 'odometry | slam | localization | map'),
    ('factor_sensor', str, False, 'A: 2d | 3d'),
    ('factor_wheel_prior', str, False, 'B: on | off'),
    ('factor_dof', str, False, 'C: se2 | se3'),
    ('factor_wheel_quality', str, False, 'D: nominal | degraded'),
    ('factor_aisle_end', str, False, 'E: open | blocked'),
    ('metric', str, True, '지표 이름 (METRICS 참고)'),
    ('param', str, False, '지표 매개변수 (예: delta=5m, tau=0.05, band=0.05-0.40, gt=slice0.20@0.01)'),
    ('alignment', str, True, 'none | origin | se2 | n/a'),
    ('segment', str, False, 'all | 정지 | 직선 | 회전 | aisle_axis | aisle_lateral | …'),
    ('stat', str, True, 'rmse | mean | median | std | p95 | max | value | n'),
    ('value', float, True, '값 (실패면 NaN 가능)'),
    ('unit', str, True, 'm | deg | % | ratio | count | s'),
    ('n', int, True, '그 값을 만든 표본 수'),
    ('failed', int, True, '0 | 1 (실패 실행 — 평균에서 빼지 않는다)'),
    ('git_head', str, True, '평가한 코드의 커밋'),
    ('gt_sha256', str, False, 'GT 를 만든 SDF 의 sha256 (gt_meta.json sdf_sha256)'),
    ('tool', str, True, '평가 도구와 판'),
    ('notes', str, False, '자유 메모 (쉼표 가능 — CSV 인용)'),
)
FIELDS = tuple(c[0] for c in COLUMNS)

ENUMS = {
    'stage': {'odometry', 'slam', 'localization', 'map'},
    'factor_sensor': {'', '2d', '3d'},
    'factor_wheel_prior': {'', 'on', 'off'},
    'factor_dof': {'', 'se2', 'se3'},
    'factor_wheel_quality': {'', 'nominal', 'degraded'},
    'factor_aisle_end': {'', 'open', 'blocked'},
    'alignment': {'none', 'origin', 'se2', 'n/a'},
    'stat': {'rmse', 'mean', 'median', 'std', 'p95', 'max', 'value', 'n'},
    'unit': {'m', 'deg', '%', 'ratio', 'count', 's'},
    'failed': {0, 1},
}
SCENARIO_RE = re.compile(r'^(S[1-7]|MAP|SYN)$')

# 이 도구가 내는 지표 이름 (다른 이름도 허용하지만 이 목록이 주 결과다)
METRICS = (
    'ate_trans', 'ate_yaw', 'rpe_trans', 'rpe_trans_pct', 'rpe_rot', 'rte_trans', 'rte_rot',
    'loc_trans', 'map_adnn0', 'map_chamfer0', 'map_precision', 'map_recall', 'map_f1',
    'map_iou', 'cloud_precision', 'cloud_recall', 'cloud_fscore', 'cloud_ac', 'cloud_com',
    'cloud_chamfer_sum', 'cloud_chamfer_half', 'cloud_acc', 'cloud_comp', 'ghost_ratio',
    'unexplained_ratio', 'dyn_sa', 'dyn_da', 'dyn_aa', 'z_err', 'roll_err', 'pitch_err',
)


class SchemaError(ValueError):
    """결과 행이 스키마에 맞지 않는다."""


def validate(row: Dict[str, object]) -> Dict[str, object]:
    """행 하나를 검사하고 형을 맞춘 사본을 돌려준다 (모르는 열·빠진 필수 열·열거값 위반은 SchemaError)."""
    unknown = set(row) - set(FIELDS)
    if unknown:
        raise SchemaError(f'모르는 열 {sorted(unknown)}')
    out: Dict[str, object] = {}
    for name, typ, required, _ in COLUMNS:
        v = row.get(name, '')
        if v is None or v == '':
            if required and not (name == 'value' and row.get('failed') in (1, '1')):
                raise SchemaError(f'필수 열 {name} 이 비었다')
            out[name] = math.nan if name == 'value' else ''
            continue
        try:
            out[name] = typ(v)
        except (TypeError, ValueError) as exc:
            raise SchemaError(f'{name}={v!r} 는 {typ.__name__} 가 아니다') from exc
        if name in ENUMS and out[name] not in ENUMS[name]:
            raise SchemaError(f'{name}={out[name]!r} 허용값 {sorted(map(str, ENUMS[name]))}')
    if not SCENARIO_RE.match(str(out['scenario'])):
        raise SchemaError(f"scenario={out['scenario']!r} (S1–S7 | MAP | SYN)")
    if out['schema'] != SCHEMA_VERSION:
        raise SchemaError(f"schema={out['schema']} (이 도구는 {SCHEMA_VERSION})")
    if out['n'] < 0:
        raise SchemaError('n < 0')
    if (not out['failed'] and out['n'] > 0 and isinstance(out['value'], float)
            and math.isnan(out['value'])):
        raise SchemaError('표본이 있는(n > 0) 정상 행의 value 가 NaN')
    return out


def rows_from_stats(common: Dict[str, object], metric: str, st: Dict[str, float], unit: str,
                    stats: Iterable[str] = ('rmse', 'mean', 'median', 'p95', 'max'),
                    **extra: object) -> List[Dict[str, object]]:
    """se2.stats() 결과를 통계마다 한 행으로 (n 은 st['n'])."""
    n = int(st.get('n', 0))
    rows = []
    for s in stats:
        row = dict(common, metric=metric, stat=s, value=st[s], unit=unit, n=n, **extra)
        row.setdefault('schema', SCHEMA_VERSION)
        rows.append(row)
    return rows


def value_row(common: Dict[str, object], metric: str, value: float, unit: str, n: int,
              **extra: object) -> Dict[str, object]:
    """값 하나짜리 행 (stat = value)."""
    row = dict(common, metric=metric, stat='value', value=value, unit=unit, n=n, **extra)
    row.setdefault('schema', SCHEMA_VERSION)
    return row


def write_csv(path: str, rows: Iterable[Dict[str, object]], append: bool = True) -> int:
    """검사한 행을 CSV 에 쓴다 (append 이고 파일이 있으면 머리줄 없이 덧붙인다). 반환: 쓴 행 수."""
    import os
    checked = [validate(r) for r in rows]
    exists = append and os.path.isfile(path) and os.path.getsize(path) > 0
    if exists:
        with open(path, encoding='utf-8', newline='') as f:
            head = next(csv.reader(f), [])
        if tuple(head) != FIELDS:
            raise SchemaError(f'{path}: 기존 머리줄이 스키마와 다르다')
    with open(path, 'a' if exists else 'w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if not exists:
            w.writeheader()
        for r in checked:
            w.writerow({k: (repr(v) if isinstance(v, float) else v) for k, v in r.items()})
    return len(checked)


def read_csv(path: str, check: bool = True) -> List[Dict[str, object]]:
    """CSV → 행 목록 (check 면 행마다 validate)."""
    with open(path, encoding='utf-8', newline='') as f:
        reader = csv.DictReader(f)
        if tuple(reader.fieldnames or ()) != FIELDS:
            raise SchemaError(f'{path}: 머리줄이 스키마와 다르다')
        rows = list(reader)
    return [validate(r) for r in rows] if check else rows


def common_fields(run_id: str, scenario: str, bag_id: str, replay: int, row: str, stage: str,
                  git_head: str, tool: str, alignment: str = 'n/a',
                  factors: Optional[Dict[str, str]] = None, **extra: object) -> Dict[str, object]:
    """행마다 같은 열 묶음. factors 키는 sensor / wheel_prior / dof / wheel_quality / aisle_end."""
    out: Dict[str, object] = {'schema': SCHEMA_VERSION, 'run_id': run_id, 'scenario': scenario,
                              'bag_id': bag_id, 'replay': replay, 'row': row, 'stage': stage,
                              'git_head': git_head, 'tool': tool, 'alignment': alignment,
                              'failed': 0}
    for k, v in (factors or {}).items():
        out[f'factor_{k}'] = v
    out.update(extra)
    return out
