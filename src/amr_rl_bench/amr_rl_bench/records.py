"""
결과 행 — gzbench/v1 (넓은 표, 이 하네스 고유) 과 rl/eval 의 episode/v1 (단계 C 판정 도구 입력) 투영 (rclpy 비의존).

episode/v1 의 열·허용 값은 `rl/eval/amr_rl_eval/schema.py` 가 규범이다. 여기서는 같은 이름으로 채우고, 그 모듈을
찾을 수 있으면(저장소 안) `validate_row` 로 검사한다 (`test/test_records.py`).
episode/v1 의 outcome 은 계약 종단 사건(metrics.outcome_contract), contacts 는 그 사건까지의 접촉 수
(contacts_contract — "성공인데 접촉" 모순 검사와 맞추려고), time_s 는 그 사건 시각이다.
"""

import csv
import json
import math
import os

SCHEMA = 'amr_rl_bench/gzbench/v1'
EPISODE_V1 = 'amr_rl_eval/episode/v1'

# gzbench/v1 열 순서 (빈 칸 = 그 제어기에 해당 없음 또는 측정 실패 — 이유는 setup_error·note 에)
COLUMNS = [
    'schema', 'run_tag', 'exec_index', 'scenario_id', 'scenario_set', 'scenario_seed',
    'difficulty',
    'controller', 'arm', 'policy_id', 'handover_radius', 'commit', 'rtf_target', 'started_wall',
    'setup_ok', 'setup_error', 'teleport_err_m', 'seed_tries', 'loc_err_start_m',
    'loc_err_start_yaw', 'accept_tries', 'path_ref_m', 'time_limit_s', 'goal_straight_m',
    'outcome_contract', 'contract_note', 'outcome_system', 'nav2_status', 'cancel_reason',
    't_contract_s', 'nav_time_s', 't_strict_gt_s', 't_strict_est_s', 't_stall_s',
    'strict_gt', 'strict_est', 'nav2compat_gt', 'nav2compat_est', 'system_success',
    'final_ok_gt', 'final_ok_est', 'final_dist_gt_m', 'final_dist_est_m', 'final_yaw_err_gt',
    'final_yaw_err_est', 'final_speed_gt', 'min_dist_gt_m', 'success_while_rotating',
    'w_at_success',
    'path_length_m', 'plan_first_m', 'n_plans', 'v_max_gt', 'w_max_gt',
    'cte_first_mean_m', 'cte_first_p95_m', 'cte_first_max_m', 'cte_active_mean_m',
    'cte_active_p95_m', 'cte_active_max_m', 'max_dev_m', 'return_max_s', 'dev_episodes',
    'interventions', 'intervention_time_s', 'zone_warning_s', 'zone_critical_s', 'zone_stop_s',
    'jerk_lin_rms', 'jerk_ang_rms',
    'contacts_static', 'contacts_dynamic', 'contacts_contract', 'min_clear_static_m',
    'min_clear_dynamic_m',
    'near_goal_stop_s', 'near_goal_stop_max_s',
    'loc_err_max_m', 'loc_err_p95_m', 'loc_err_end_m', 'loc_err_end_yaw', 'loc_collapse',
    'lost_events', 'lost_at_start',
    'decisions', 'src_policy', 'src_fallback_cell', 'src_dwa', 'src_align', 'dwa_state',
    'dwa_policy_error', 'dwa_timeout', 'dwa_nonfinite', 'dwa_handover',
    'decision_period_mean_s', 'decision_period_p05_s', 'decision_period_p95_s',
    'tick_dt_sim_mean', 'ctrl_dt_sim_mean', 'ctrl_dt_sim_p05', 'ctrl_dt_sim_p95',
    'ctrl_dt_wall_mean', 'infer_ms_p50', 'infer_ms_p95', 'infer_ms_max', 'cycle_ms_p99',
    'scan_age_p50_s', 'scan_age_p95_s', 'scan_to_cmd_p50_s', 'scan_to_cmd_p95_s',
    'chain_lag_p50_s', 'chain_lag_p95_s',
    'rtf', 'wall_setup_s', 'wall_nav_s', 'wall_episode_s', 'load_start', 'load_end', 'note',
]

ARM_OF = {'dwa': 'A_dwa', 'rl': 'B_ppo', 'rl_handover': 'B_ppo'}


def scenario_set_name(which):
    """집합 이름('main' | 'stress') → episode/v1 scenario_set 이름."""
    return f'gz_static_v1_{which}'


def fmt(v):
    """CSV 칸 (None → 빈 칸, bool → 0/1, float → 6 자리)."""
    if v is None:
        return ''
    if isinstance(v, bool):
        return int(v)
    if isinstance(v, float):
        return '' if not math.isfinite(v) else round(v, 6)
    return v


def gzbench_row(meta, metrics):
    """meta(실행·시나리오 정보) + metrics(episode_metrics) → gzbench/v1 dict (열 순서는 COLUMNS)."""
    row = {k: None for k in COLUMNS}
    row.update({k: v for k, v in meta.items() if k in row})
    row.update({k: v for k, v in metrics.items() if k in row})
    row['schema'] = SCHEMA
    return row


def episode_v1_row(g, episode_index=0, train_seed=None):
    """gzbench/v1 행 → episode/v1 행 (rl/eval schema 의 열 이름)."""
    setup_ok = bool(g.get('setup_ok'))
    outcome = g.get('outcome_contract') if setup_ok else 'error'
    contacts = int(g.get('contacts_contract') or 0) if outcome != 'error' else 0
    if outcome == 'collision' and contacts == 0:
        contacts = 1
    clear = [x for x in (g.get('min_clear_static_m'), g.get('min_clear_dynamic_m'))
             if x is not None]
    return {
        'schema_version': EPISODE_V1,
        'arm': g.get('arm'),
        'policy_id': g.get('policy_id'),
        'train_seed': train_seed,
        'simulator': 'gazebo',
        'scenario_set': scenario_set_name(g.get('difficulty')),
        'scenario_id': g.get('scenario_id'),
        'scenario_seed': g.get('scenario_seed'),
        'difficulty': g.get('difficulty'),
        'episode': episode_index,
        'run_tag': g.get('run_tag'),
        'exec_index': g.get('exec_index'),
        'commit': g.get('commit') or 'unknown',
        'safety_node': 'on',
        'outcome': outcome,
        'contacts': contacts,
        'time_s': g.get('t_contract_s') if g.get('t_contract_s') is not None else 0.0,
        'path_length_m': g.get('path_length_m'),
        'cte_mean_m': g.get('cte_active_mean_m'),
        'cte_p95_m': g.get('cte_active_p95_m'),
        'max_dev_m': g.get('max_dev_m'),
        'return_max_s': g.get('return_max_s'),
        'min_clearance_m': min(clear) if clear else None,
        'min_ttc_s': None,
        'interventions': g.get('interventions'),
        'intervention_time_s': g.get('intervention_time_s'),
        'jerk_lin_rms': g.get('jerk_lin_rms'),
        'jerk_ang_rms': g.get('jerk_ang_rms'),
        'fallback_count': g.get('src_dwa') if g.get('controller') != 'dwa' else None,
        'infer_ms_p95': g.get('infer_ms_p95'),
        'rtf': g.get('rtf'),
    }


def append_csv(path, columns, row):
    """CSV 에 한 행 덧붙인다 (파일이 없으면 머리말부터)."""
    new = not os.path.exists(path)
    with open(path, 'a', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=columns)
        if new:
            w.writeheader()
        w.writerow({k: fmt(row.get(k)) for k in columns})


def write_json(path, obj):
    """JSON (UTF-8, NaN/inf → null)."""
    def clean(x):
        if isinstance(x, float) and not math.isfinite(x):
            return None
        if isinstance(x, dict):
            return {k: clean(v) for k, v in x.items()}
        if isinstance(x, (list, tuple)):
            return [clean(v) for v in x]
        return x
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(clean(obj), f, ensure_ascii=False)
