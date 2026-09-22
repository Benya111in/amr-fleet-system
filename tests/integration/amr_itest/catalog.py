"""
시나리오 목록 (명세 4.10 "통합 테스트 시나리오 ≥ 10" — tests/integration/README.md 표의 단일 출처).

각 test_NN_<slug>.py 는 여기서 자기 Scenario 를 꺼내 쓴다 (CTX = Context(catalog.get(N))).
scripts/run_integration.sh --list 와 report.py 의 요약 표도 이 목록을 읽는다. 시나리오를 추가할 때는
여기에 한 항목 + test_NN_<slug>.py 한 파일을 더한다.
"""

from typing import Dict, List

from amr_itest.scenario import COMPONENT, GAZEBO, KINEMATIC, Scenario, SYSTEM

SCENARIOS: List[Scenario] = [
    Scenario(
        number=1, slug='sensor_topics', title='시뮬레이션 기동 및 센서 토픽 발행',
        spec='4.1 센서 시스템 구성 (LiDAR 10 Hz · depth 15 Hz · RGB 30 Hz · IMU 100 Hz)',
        metric='센서 토픽 header.stamp(sim time) 주기: 평균 (n−1)/span, 중앙값 1/median(Δt); '
               '메시지 규격(빔 수·해상도·FOV·거리), 정지 상태 노이즈 σ',
        threshold='토픽마다 평균 ≥ 0.9×규정, 중앙값 ≥ 0.95×규정; 규격 일치; LiDAR σ ∈ [0.5, 2]×0.03 m',
        log_format='topic_rates.csv [topic, type, nominal_hz, count, mean_rate_hz, '
                   'median_rate_hz, max_gap_s, wall_rate_hz, frame_id, ok]',
        backends=(GAZEBO,)),
    Scenario(
        number=2, slug='tf_tree', title='TF 트리 무결성',
        spec='4.1 로봇 모델링 (센서 위치 = TF), 9장 "TF 트리 문서화"',
        metric='/tf·/tf_static 간선 그래프 vs 계약 간선(map→odom→base_footprint→base_link→센서): '
               '존재·부모 유일성·순환·정적 변환 값(config extrinsic)·동적 간선 주기',
        threshold='누락 간선 0, 이중 부모 0, 순환 0, 루트 = map 하나, 정적 변환 오차 ≤ 1 mm / 0.1°, '
                  'EKF 간선 ≥ 0.9×50 Hz',
        log_format='tf_edges.csv [parent, child, publisher, kind, present, translation_error_m, '
                   'rotation_error_deg, rate_hz, ok, problems] + frames.dot',
        backends=(GAZEBO, KINEMATIC)),
    Scenario(
        number=3, slug='slam_map', title='SLAM 맵 생성',
        spec='4.3 SLAM (해상도 ≤ 0.05 m, 구조물 일치)',
        metric='slam_toolbox /map 해상도·크기 + 월드 SDF 정적 구조물(벽·랙·기둥) 점유 셀 일치율',
        threshold='resolution ≤ 0.05 m, 구조물 셀 재현율 ≥ 0.9, 자유 공간 오검출 ≤ 5 %',
        log_format='map_check.json {resolution, width, height, recall, false_occupied} + map.pgm',
        backends=(GAZEBO,), profiles=(SYSTEM,), implemented=False),
    Scenario(
        number=4, slug='ekf_accuracy', title='EKF 위치 추정 정확도',
        spec='4.3 위치 추정 (정지 3 / 직선 5 / 회전 8 cm), 4.10 위치 추정 RMSE',
        metric='odometry/filtered_map vs ground_truth/odom (GT 를 추정 스탬프에 보간), '
               'GT 속도로 정지/직선/회전 분류, 구간별 RMSE·최대',
        threshold='RMSE 정지 ≤ 0.03 m, 직선 ≤ 0.05 m, 회전 ≤ 0.08 m; 구간별 ≥ 100 샘플',
        log_format='pose_error.csv [timestamp, gt_x, gt_y, est_x, est_y, error, yaw_error, '
                   'segment] (amr_evaluation pose_error_logger) + harness_pose_error.csv',
        backends=(KINEMATIC, GAZEBO)),
    Scenario(
        number=5, slug='kidnapped_robot', title='Kidnapped Robot 복구',
        spec='4.3 위치 추정 (납치 후 제자리 회전 등으로 자가 복구)',
        metric='Gazebo set_pose 로 로봇 순간 이동 → localization/lost 전이 → 복구 후 '
               'odometry/filtered_map vs GT 오차, 복구 시간',
        threshold='lost 감지 ≤ 5 s, 복구(오차 ≤ 0.10 m 로 3 s 유지) ≤ 60 s, 5회 중 5회 성공',
        log_format='kidnap.csv [trial, from_x, from_y, to_x, to_y, detect_s, recover_s, '
                   'final_error_m, ok]',
        backends=(GAZEBO,), profiles=(SYSTEM,), implemented=False),
    Scenario(
        number=6, slug='path_planning', title='경로 계획 성공률',
        spec='4.4 전역 경로 계획 (임의 50쌍 성공률 ≥ 98 %)',
        metric='자유 셀에서 뽑은 50 쌍(시드 고정)에 compute_path_to_pose(A* 플러그인) 호출, '
               '성공 여부·계획 시간·경로 길이/직선 거리',
        threshold='성공률 ≥ 98 % (50쌍 중 ≥ 49), 경로가 점유 셀을 지나지 않음',
        log_format='plans.csv [pair, start_x, start_y, goal_x, goal_y, success, plan_ms, '
                   'length_m, straight_m, collision_free]',
        backends=(GAZEBO,), profiles=(SYSTEM,), implemented=False),
    Scenario(
        number=7, slug='path_tracking_cte', title='경로 추종 CTE',
        spec='4.5 경로 추종 (CTE 직선 5 cm / 곡선 10 cm), 4.10 CTE 산출',
        metric='navigate_to_pose 주행 중 plan vs ground_truth 수직 거리, 곡률로 직선/곡선 분리 평균',
        threshold='평균 |CTE| 직선 ≤ 0.05 m, 곡선 ≤ 0.10 m',
        log_format='cte.csv [timestamp, planned_x, planned_y, actual_x, actual_y, cte, segment]'
                   ' (amr_evaluation cte_logger)',
        backends=(GAZEBO,), profiles=(SYSTEM,), implemented=False),
    Scenario(
        number=8, slug='dynamic_obstacles', title='동적 장애물 회피',
        spec='4.7 동적 장애물 (30회 충돌 0건, 경로 이탈 1 m 이내)',
        metric='장애물 교차 시나리오 30회: GT 풋프린트 ~ 장애물 최소 여유, 계획 경로 대비 최대 이탈',
        threshold='충돌(여유 ≤ 0) 0건 / 30회, 최대 이탈 ≤ 1.0 m, 목표 도달 ≥ 29회',
        log_format='avoidance.csv [trial, pattern, speed_mps, min_clearance_m, max_deviation_m, '
                   'reached, time_s]',
        backends=(GAZEBO,), profiles=(SYSTEM,), implemented=False),
    Scenario(
        number=9, slug='emergency_stop', title='긴급 정지',
        spec='4.7 안전 (0.3 m 이내 접근 시 즉시 정지), 4.9 E-stop 알림',
        metric='(a) 장애물 주입 → 첫 cmd_vel=0 지연(첫 침범 스캔 수신 기준) '
               '(b) E-stop 버튼(estop, /fleet/estop) → cmd_vel=0 지연 '
               '(c) 벽 접근 폐루프: 정지 명령 시점 GT 여유, 최종 여유',
        threshold='지연 최댓값 ≤ 100 ms (외부 부하 시 p95, 20회/종류); 정지 시점 GT 여유 '
                  '≥ 0.25 m, 충돌 0; '
                  'E-stop 래치는 reset_estop 전까지 유지',
        log_format='estop_latency.csv [trial, kind, t_trigger, t_zero, latency_ms, '
                   'clearance_m] + approach.csv',
        backends=(KINEMATIC, GAZEBO)),
    Scenario(
        number=10, slug='docking', title='정밀 도킹',
        spec='4.8 도킹 (위치 2 cm / 각도 1° 이내)',
        metric='dock 액션 10회: 결과 final_position_error/final_angle_error + GT 로 독립 재측정',
        threshold='위치 ≤ 0.02 m, 각도 ≤ 1°(0.01745 rad), 10회 중 10회',
        log_format='docking.csv [trial, dock_id, success, attempts, pos_err_m, ang_err_deg, '
                   'gt_pos_err_m, gt_ang_err_deg, time_s]',
        backends=(GAZEBO,), profiles=(SYSTEM,), implemented=False),
    Scenario(
        number=11, slug='bt_recovery', title='BT 에러 복구',
        spec='4.8 행동 트리 (3가지 이상 에러 상황 자동 복구)',
        metric='작업 중 에러 주입 3종(E-stop 래치, 경로 차단, 위치 상실) → task_status 최종 COMPLETED, '
               'executor/phase 의 error→복구 전이',
        threshold='3종 모두 작업 완료(COMPLETED), 복구 ≤ 60 s',
        log_format='recovery.csv [case, injected_at, recovered_at, recovery_s, final_status, '
                   'phases]',
        backends=(GAZEBO,), profiles=(SYSTEM,), implemented=False),
    Scenario(
        number=12, slug='multi_robot_deadlock', title='5대 동시 운용 교착',
        spec='4.9 교통 관리 (교착 탐지·해소), 4.10 CPU ≤ 80 %',
        metric='좁은 통로 맞교차 작업 → /fleet/traffic_events 교착 탐지·해소, 작업 완료, '
               'cpu_sampler 평균 CPU',
        threshold='교착 탐지 ≥ 1 & 전부 해소, 5대 작업 모두 완료, 평균 CPU ≤ 80 %',
        log_format='traffic.csv [time, event, robots] + cpu.csv (amr_evaluation cpu_sampler)',
        backends=(GAZEBO,), profiles=(SYSTEM,), implemented=False),
    Scenario(
        number=13, slug='response_time', title='응답 시간',
        spec='4.10 성능 (명령 수신 ~ 로봇 반응 평균 200 ms, 50회 이상), sequences.md §1 성능 표',
        metric='assign_task 요청(header.stamp) → 첫 cmd_vel ≠ 0 (sequences.md §1 정의), 50회 '
               '평균/최대; 참고로 ground_truth/odom 첫 |v| ≥ 0.05 m/s 까지 (판정 안 함)',
        threshold='평균 ≤ 200 ms, 샘플 ≥ 50, 무응답(cmd_vel·GT 움직임) 0',
        log_format='response_time.csv [cmd_time, response_time, latency_ms, cmd_id] '
                   '(amr_evaluation response_time_logger, twist 모드) + harness_response_time.csv '
                   '+ harness_motion_onset.csv',
        backends=(KINEMATIC, GAZEBO)),
    Scenario(
        number=14, slug='soak', title='4시간 연속 운용',
        spec='4.10 안정성 (연속 4시간, 크래시·메모리 누수 없음)',
        metric='system 스택에 작업을 반복 투입하며 프로세스별 RSS 샘플링(60 s), 선형 회귀 기울기, '
               '프로세스 종료·재시작 감시',
        threshold='크래시 0, 프로세스별 RSS 증가율 ≤ 5 MB/h (첫 10 분 워밍업 제외), 작업 실패율 ≤ 2 %',
        log_format='memory.csv [time, process, pid, rss_mb] + soak.json',
        backends=(GAZEBO,), profiles=(SYSTEM,), implemented=False, long_running=True),
]

_BY_NUMBER: Dict[int, Scenario] = {s.number: s for s in SCENARIOS}


def get(number: int) -> Scenario:
    """번호로 시나리오를 찾는다 (없으면 KeyError)."""
    return _BY_NUMBER[number]


def by_id(scenario_id: str) -> Scenario:
    """'09_emergency_stop' 또는 '9' 또는 'emergency_stop' 으로 찾는다."""
    key = scenario_id.strip()
    if key.isdigit():
        return get(int(key))
    for s in SCENARIOS:
        if key in (s.id, s.slug, f'test_{s.id}', f'test_{s.id}.py'):
            return s
    raise KeyError(scenario_id)


def table_markdown() -> str:
    """tests/integration/README.md 의 시나리오 표 (지표·기준·로그 포맷·상태)."""
    def cell(text: str) -> str:
        return text.replace('|', '\\|')

    lines = ['| # | 시나리오 | 지표 | 합격 기준 | 로그 | 백엔드 | 상태 |',
             '| --- | --- | --- | --- | --- | --- | --- |']
    for s in SCENARIOS:
        state = '구현' if s.implemented else '스켈레톤 (필요 노드 부재 시 skip)'
        if s.long_running:
            state += ', 장시간 (명시 선택 시만)'
        lines.append(f'| {s.number} | {s.title} | {cell(s.metric)} | {cell(s.threshold)} | '
                     f'`{s.log_format}` | {", ".join(s.backends)} | {state} |')
    return '\n'.join(lines) + '\n'


__all__ = ['COMPONENT', 'SCENARIOS', 'by_id', 'get', 'table_markdown']
