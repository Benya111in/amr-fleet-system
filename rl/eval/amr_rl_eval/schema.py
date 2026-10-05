"""
결과 CSV 계약 — 에피소드 한 행 (schema `amr_rl_eval/episode/v1`).

r1(Isaac)·r2(2D)·r3(Gazebo 플러그인) 과 Gazebo 기준선(DWA) 이 **같은 열**로 결과를 낸다.
열의 의미·단위·필수 여부는 PROTOCOL.md §5 표가 규범이고, 이 모듈은 그 표를 코드로 옮긴 것이다
(둘이 어긋나면 시험 `test_schema.py::test_protocol_table_matches_schema` 가 깨진다).

판정 규칙과 맞추는 정의 (rl/contract 의 reward.py 와 같은 이름·문턱):
  outcome     success | collision | timeout | stall | error
              success  = 목표 xy ≤ 0.10 m 이고 |v| ≤ 0.05 m/s (계약 is_success), 접촉과 동시면 collision
              timeout  = 시간 상한 (계약 time_limit) / stall = 진행 정체 (계약 ProgressChecker)
              error    = 하네스·시뮬레이터 고장 — **분석에서 빼지 않고** 따로 센다 (PROTOCOL.md §6.6)
  contacts    지면 진실 접촉 사건 수 (Gazebo collision_monitor, 2D·Isaac 은 풋프린트 대 장애물 기하)
  max_dev_m   그 시각 유효한(활성) 계획 대비 최대 이탈 (metrics.deviation_to_active_plan 과 같은 정의)
  return_max_s  이탈 0.30 m 구간의 최대 이탈 시각 → 0.15 m 복귀까지 최대 시간
  interventions  safety node 가 명령을 바꾼 사건 수 (입력 ≠ 출력이 연속된 구간 하나 = 1 회)
"""

import csv
import math

SCHEMA_VERSION = 'amr_rl_eval/episode/v1'

ARMS = ('A_dwa', 'B_ppo', 'B_sac', 'C_hybrid', 'D_param')
SIMULATORS = ('sim2d', 'isaac', 'gazebo')
OUTCOMES = ('success', 'collision', 'timeout', 'stall', 'error')
SAFETY_MODES = ('on', 'off', 'monitor')

# (이름, 형, 필수, 단위·설명)
COLUMNS = [
    ('schema_version', str, True, f'항상 {SCHEMA_VERSION}'),
    ('arm', str, True, '비교군 — ' + ' · '.join(ARMS)),
    ('policy_id', str, True, '체크포인트 식별자 (예: C_hybrid/s3/2.0M) · DWA 는 dwa@<커밋>'),
    ('train_seed', int, False, '학습 시드 (DWA 는 빈 칸)'),
    ('simulator', str, True, ' · '.join(SIMULATORS)),
    ('scenario_set', str, True, '시나리오 묶음 이름 (PROTOCOL.md §3)'),
    ('scenario_id', str, True, '시나리오·레벨 식별자 (CVaR 의 레벨 단위)'),
    ('scenario_seed', int, True, '에피소드 시나리오 시드 — **짝짓기 키**'),
    ('difficulty', str, True, '난도 층 (rliable task 단위)'),
    ('episode', int, True, '같은 (policy_id, scenario_seed) 안의 순번 (0 부터)'),
    ('run_tag', str, True, '로그 태그·실행 디렉터리'),
    ('exec_index', int, False, '같은 (arm, simulator, scenario_set) 안의 실행 순번 — 순차 감시 순서 '
                               '(Gazebo 확증 평가에서는 필수, PROTOCOL.md §6.2)'),
    ('commit', str, True, 'git rev-parse HEAD (모르면 unknown — 판정표에 표시된다)'),
    ('safety_node', str, True, ' · '.join(SAFETY_MODES)),
    ('outcome', str, True, ' · '.join(OUTCOMES)),
    ('contacts', int, True, '지면 진실 접촉 사건 수'),
    ('time_s', float, True, '시작→종료 시뮬레이션 시간 [s]'),
    ('path_length_m', float, False, '실제 주행 거리 [m]'),
    ('cte_mean_m', float, False, '추종 모드 CTE 절댓값 평균 [m]'),
    ('cte_p95_m', float, False, '추종 모드 CTE 절댓값 95 분위 [m]'),
    ('max_dev_m', float, False, '활성 계획 대비 최대 이탈 [m]'),
    ('return_max_s', float, False, '이탈 구간 최대 복귀 시간 [s] (구간 없으면 0)'),
    ('min_clearance_m', float, False, '동적·정적 장애물 최소 부호 거리 [m]'),
    ('min_ttc_s', float, False, '등속 가정 최소 TTC [s] (조우 없으면 inf)'),
    ('interventions', int, False, 'safety node 개입 사건 수'),
    ('intervention_time_s', float, False, '개입 지속 합 [s]'),
    ('jerk_lin_rms', float, False, '선 저크 RMS [m/s³]'),
    ('jerk_ang_rms', float, False, '각 저크 RMS [rad/s³]'),
    ('fallback_count', int, False, '하이브리드 C 폴백 횟수 (그 밖 비교군은 빈 칸)'),
    ('infer_ms_p95', float, False, '제어 주기당 추론 지연 95 분위 [ms]'),
    ('rtf', float, False, 'Gazebo 실시간 비율 (공유 부하 하 예비값)'),
]

NAMES = [c[0] for c in COLUMNS]
REQUIRED = [c[0] for c in COLUMNS if c[2]]
TYPES = {c[0]: c[1] for c in COLUMNS}


def _parse(name, raw):
    t = TYPES[name]
    if raw is None or raw == '' or (isinstance(raw, float) and math.isnan(raw)):
        return None
    if t is str:
        return str(raw)
    if t is int:
        f = float(raw)
        if not f.is_integer():
            raise ValueError(f'{name}: 정수가 아니다 ({raw})')
        return int(f)
    return float(raw)


def validate_row(row):
    """
    한 행 검사 → (정규화한 dict, 오류 문자열 리스트).

    누락 선택 열은 None. 오류가 있어도 가능한 만큼 정규화한다.
    """
    out, errs = {}, []
    for name in NAMES:
        try:
            out[name] = _parse(name, row.get(name))
        except (TypeError, ValueError) as e:
            errs.append(str(e))
            out[name] = None
    for name in REQUIRED:
        if out.get(name) is None:
            errs.append(f'필수 열 비어 있음: {name}')
    if out.get('schema_version') not in (None, SCHEMA_VERSION):
        errs.append(f'schema_version 불일치: {out["schema_version"]}')
    for name, allowed in (('arm', ARMS), ('simulator', SIMULATORS), ('outcome', OUTCOMES),
                          ('safety_node', SAFETY_MODES)):
        if out.get(name) is not None and out[name] not in allowed:
            errs.append(f'{name} 값이 허용 목록 밖: {out[name]}')
    c = out.get('contacts')
    if c is not None and c < 0:
        errs.append('contacts < 0')
    if out.get('outcome') == 'success' and c:
        errs.append('outcome=success 인데 contacts > 0 (계약: 접촉과 동시면 collision)')
    if out.get('outcome') == 'collision' and c == 0:
        errs.append('outcome=collision 인데 contacts = 0')
    extra = set(row) - set(NAMES)
    if extra:
        errs.append(f'모르는 열: {sorted(extra)}')
    return out, errs


def read_csv(path, strict=True):
    """
    결과 CSV 읽기 → 정규화한 dict 리스트.

    strict=True 면 오류가 하나라도 있으면 ValueError (행 번호와 함께).
    """
    rows, problems = [], []
    with open(path, newline='', encoding='utf-8') as f:
        for i, raw in enumerate(csv.DictReader(f), start=2):
            row, errs = validate_row(raw)
            rows.append(row)
            problems.extend(f'{path}:{i}: {e}' for e in errs)
    if strict and problems:
        raise ValueError('\n'.join(problems[:50]))
    keys = [(r['policy_id'], r['simulator'], r['scenario_set'], r['scenario_seed'], r['episode'],
             r['safety_node']) for r in rows]
    if strict and len(set(keys)) != len(keys):
        raise ValueError(f'{path}: (policy_id, simulator, scenario_set, scenario_seed, episode, '
                         'safety_node) 중복')
    return rows


def write_csv(path, rows):
    """정규화 dict 리스트 → CSV (열 순서 = COLUMNS, None 은 빈 칸)."""
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=NAMES)
        w.writeheader()
        for r in rows:
            w.writerow({k: ('' if r.get(k) is None else r.get(k)) for k in NAMES})


def success(row):
    """성공 여부 (outcome 에서 유도 — 따로 저장하지 않는다)."""
    return row['outcome'] == 'success'


def collided(row):
    """접촉 여부 (contacts > 0)."""
    return (row['contacts'] or 0) > 0


def markdown_table():
    """PROTOCOL.md §5 에 싣는 열 표 (마크다운)."""
    lines = ['| 열 | 형 | 필수 | 뜻·단위 |', '| --- | --- | --- | --- |']
    for name, t, req, desc in COLUMNS:
        lines.append(f'| `{name}` | {t.__name__} | {"예" if req else ""} | {desc} |')
    return '\n'.join(lines)
