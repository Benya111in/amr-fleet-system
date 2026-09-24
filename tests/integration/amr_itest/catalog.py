"""
시나리오 목록 (명세 4.10 "통합 테스트 시나리오 ≥ 10" — tests/integration/README.md 표의 단일 출처).

각 test_NN_<slug>.py 는 여기서 자기 Scenario 를 꺼내 쓴다 (CTX = Context(catalog.get(N))).
scripts/run_integration.sh --list 와 report.py 의 요약 표도 이 목록을 읽는다. 시나리오를 추가할 때는
여기에 한 항목 + test_NN_<slug>.py 한 파일을 더한다.

needs: 시나리오가 쓰는 워크스페이스 패키지 — 모두 설치돼 있는데 skip 되면 집계가 실패로 센다.
map 프레임 = 월드 프레임 (maps/warehouse.yaml 은 월드에 정합된 지도): system 프로필 시나리오는 그 가정을
판정 전에 확인한다 (cases.ProbeCase.check_map_registration — /map 점유 셀 ↔ 월드 visual 단면 SE(2) 정합).
"""

from typing import Dict, List

from amr_itest.scenario import COMPONENT, GAZEBO, KINEMATIC, Scenario, SYSTEM

SIM = ('amr_simulation', 'amr_description')
BRINGUP = ('amr_bringup',) + SIM
NAV = BRINGUP + ('amr_localization', 'amr_navigation', 'amr_perception')
FULL = NAV + ('amr_behavior',)

REGISTRATION = 'map↔월드 항등 정합 (잔여 이동 ≤ 5 cm, 회전 ≤ 0.1°, 거리 중앙값 ≤ 5 cm)'

SCENARIOS: List[Scenario] = [
    Scenario(
        number=1, slug='sensor_topics', title='시뮬레이션 기동 및 센서 토픽 발행',
        spec='4.1 센서 시스템 구성 (LiDAR ≥ 10 Hz · depth ≥ 15 Hz · RGB 30 Hz · IMU ≥ 100 Hz, '
             'depth 최대 10 m + 노이즈)',
        metric='센서 토픽 header.stamp(sim time) 주기: 중앙값 1/median(Δt) = 센서 주기, 평균 (n−1)/span = '
               '전달률; 메시지 규격(빔 수·해상도·FOV·거리), 정지 상태 노이즈 σ (LiDAR 빔별, IMU, depth 화소별)',
        threshold='규정 주기 = max(명세, sensors.yaml): 중앙값 ≥ 0.98×(지터만 허용), 전달률 평균 ≥ 0.9×; '
                  '규격 일치; LiDAR σ ∈ [0.5, 2]×0.03 m; depth 유효값 ≤ 10 m, 정지 화소 σ > 0',
        log_format='topic_rates.csv [topic, type, nominal_hz, count, mean_rate_hz, '
                   'median_rate_hz, max_gap_s, wall_rate_hz, frame_id, ok]',
        backends=(GAZEBO,), needs=SIM),
    Scenario(
        number=2, slug='tf_tree', title='TF 트리 무결성',
        spec='4.1 로봇 모델링 (센서 위치 = TF), 9장 "TF 트리 문서화"',
        metric='/tf·/tf_static 간선 그래프 vs 계약 간선(map→odom→base_footprint→base_link→센서): '
               '존재·부모 유일성·순환·정적 변환 값(config extrinsic)·동적 간선 주기',
        threshold='누락 간선 0, 이중 부모 0, 순환 0, 루트 = map 하나, 정적 변환 오차 ≤ 1 mm / 0.1°, '
                  'EKF 간선 ≥ 0.9×50 Hz',
        log_format='tf_edges.csv [parent, child, publisher, kind, present, translation_error_m, '
                   'rotation_error_deg, rate_hz, ok, problems] + frames.dot',
        backends=(GAZEBO, KINEMATIC), needs=('amr_description', 'amr_localization')),
    Scenario(
        number=3, slug='slam_map', title='SLAM 맵 생성',
        spec='4.3 SLAM (해상도 ≤ 0.05 m, 구조물 일치)',
        metric='slam_toolbox /map (월드 원점 스폰 → map = 월드) vs 월드 SDF visual 의 LiDAR 평면 단면: '
               '해상도, 보이는 구조물 가장자리 재현율, 자유 공간 오점유율, SE(2) 잔여 정합',
        threshold=f'resolution ≤ 0.05 m, 재현율 ≥ 0.9, 오점유 ≤ 5 %, {REGISTRATION}, save_map 성공',
        log_format='map_info.json {resolution, width, height, origin} + map.pgm/yaml (save_map) '
                   '+ map_received.pgm/yaml (판정한 /map)',
        backends=(GAZEBO,), profiles=(SYSTEM,), needs=BRINGUP + ('amr_localization',)),
    Scenario(
        number=4, slug='ekf_accuracy', title='EKF 위치 추정 정확도',
        spec='4.3 위치 추정 (정지 3 / 직선 5 / 회전 8 cm), 4.10 위치 추정 RMSE',
        metric='odometry/filtered_map vs ground_truth/odom (GT 를 추정 스탬프에 보간), '
               'GT 속도로 정지/직선/회전 분류, 구간별 RMSE·최대. system: 실제 AMCL + maps/warehouse.yaml '
               '(map↔월드 정합 확인 후) / component: AMCL 대역(GT+σ 1.5 cm) — EKF 배관 확인일 뿐 명세 판정 아님',
        threshold='RMSE 정지 ≤ 0.03 m, 직선 ≤ 0.05 m, 회전 ≤ 0.08 m; 구간별 ≥ 100 샘플; '
                  f'system: {REGISTRATION}',
        log_format='pose_error.csv [timestamp, gt_x, gt_y, est_x, est_y, error, yaw_error, '
                   'segment] (amr_evaluation pose_error_logger) + harness_pose_error.csv',
        backends=(GAZEBO, KINEMATIC), profiles=(SYSTEM, COMPONENT),
        needs=BRINGUP + ('amr_localization', 'amr_evaluation')),
    Scenario(
        number=5, slug='kidnapped_robot', title='Kidnapped Robot 복구',
        spec='4.3 위치 추정 (납치 후 제자리 회전 등으로 자가 복구)',
        metric='Gazebo set_pose 로 로봇 순간 이동 → localization/lost 전이 → 복구 후 '
               'odometry/filtered_map vs GT 오차, 복구 시간',
        threshold=f'lost 감지 ≤ 5 s, 복구(오차 ≤ 0.10 m 로 3 s 유지) ≤ 60 s, 5회 중 5회; {REGISTRATION}',
        log_format='kidnap.csv [trial, from_x, from_y, to_x, to_y, detect_s, recover_s, '
                   'final_error_m, ok] (시행마다 갱신)',
        backends=(GAZEBO,), profiles=(SYSTEM,), needs=NAV),
    Scenario(
        number=6, slug='path_planning', title='경로 계획 성공률',
        spec='4.4 전역 경로 계획 (임의 50쌍 성공률 ≥ 98 %, 재계획 500 ms)',
        metric='월드 자유 지점 50 쌍(시드 고정)에 compute_path_to_pose(planner_id=AStar, 직접 구현 A*), '
               'planner_server 활성·지도 수신 후; 성공·계획 시간(result.planning_time)·경로 길이; '
               '참고 NavFn·Smac 같은 쌍',
        threshold=f'성공률 ≥ 98 % (≥ 49/50), 경로가 구조물(visual∪collision)을 지나지 않음, 계획 시간 최대 ≤ 500 ms; '
                  f'{REGISTRATION}',
        log_format='plans.csv [pair, planner, start_x, start_y, goal_x, goal_y, success, plan_ms, '
                   'length_m, straight_m, collision_free]',
        backends=(GAZEBO,), profiles=(SYSTEM,), needs=BRINGUP + ('amr_localization',
                                                                 'amr_navigation')),
    Scenario(
        number=7, slug='path_tracking_cte', title='경로 추종 CTE',
        spec='4.5 경로 추종 (CTE 직선 5 cm / 곡선 10 cm), 4.10 CTE 산출',
        metric='navigate_to_pose 주행 중 plan(map) vs ground_truth(월드) 수직 거리 — map = 월드 정합을 먼저 확인, '
               '곡률로 직선/곡선 분리 평균 (amr_evaluation cte_logger → analyze)',
        threshold=f'평균 |CTE| 직선 ≤ 0.05 m, 곡선 ≤ 0.10 m, 목표 전부 도달; {REGISTRATION}',
        log_format='cte.csv [timestamp, planned_x, planned_y, actual_x, actual_y, cte, segment]'
                   ' (amr_evaluation cte_logger)',
        backends=(GAZEBO,), profiles=(SYSTEM,), needs=NAV + ('amr_evaluation',)),
    Scenario(
        number=8, slug='dynamic_obstacles', title='동적 장애물 회피',
        spec='4.7 동적 장애물 (30회 충돌 0건, 경로 이탈 1 m 이내, 5 s 안에 원경로 복귀)',
        metric='작업자 횡단선을 지나는 왕복 30회: amr_simulation collision_monitor(사람·지게차·셔틀 모두) 접촉·'
               '최소 거리, 시행별 최근접 장애물·TTC, 첫 계획 경로 대비 GT 이탈, 이탈 > 0.3 m 구간 뒤 복귀 시간',
        threshold='접촉 0 / 30회, 최대 이탈 ≤ 1.0 m, 복귀 ≤ 5 s, 목표 도달 ≥ 29회, '
                  '실제 조우(최근접 동적 장애물 ≤ 2.0 m) ≥ 시행/6 (30회면 5), 시행 ≥ 30',
        log_format='avoidance.csv [trial, reached, time_s, contacts, min_distance_m, nearest, '
                   'min_ttc_s, max_deviation_m, episodes, return_s, contacts_robot_moving, '
                   'episodes_open, open_peak_m, open_peak_t, dev_at_end_m] (시행마다 '
                   '갱신), contacts.csv [trial, time, obstacle, distance_m, robot_speed_mps, '
                   'robot_moving, obstacle_speed_mps, obstacle_heading_deg, lane_lateral_m, '
                   'lane_along_m, yield_state, stop_distance_m] (뒤 6개는 원인 귀속 근거 — '
                   '판정에 쓰지 않는다), episodes.csv [trial, t_start, t_peak, peak_m, '
                   't_end, return_s, stopped_frac, mean_v_mps, max_v_mps, yield_frac, '
                   'cmd_v_mean, gate_out_v_mean, vo_rejected_mean] (복귀가 늦은 구간에서 '
                   '로봇이 멈춰 있었는지·무엇이 세웠는지), 접촉이 난 시행만 '
                   'stats_trial<N>.csv [t, cmd_v, cmd_w, yield_state, stop_distance_m, '
                   'vo_rejected, collisions, ttc_s, zone_entry_m, yield_obstacle] 와 '
                   'tracks_trial<N>.csv [t, track_id, x, y, vx, vy, heading_deg, is_dynamic] '
                   '(제어 이력·추적 입력 — 전부 근거일 뿐 판정에 쓰지 않는다)',
        backends=(GAZEBO,), profiles=(SYSTEM,), needs=NAV),
    Scenario(
        number=9, slug='emergency_stop', title='긴급 정지',
        spec='4.7 안전 (0.3 m 이내 접근 시 즉시 정지), 4.9 E-stop 알림, sequences.md §2 (safety_node ≤ 20 ms)',
        metric='(a) 장애물 주입 → 첫 cmd_vel=0: 침범 스캔 수신 기준(safety 처리) · 주입 기준(스캔 주기 포함) '
               '(b) E-stop 버튼(estop, /fleet/estop) → cmd_vel=0, 래치·reset (c) 벽 접근 폐루프: '
               '정지 명령 시점 GT 여유',
        threshold='최댓값으로 판정 (부하 중 초과면 한 번 재측정): 스캔→0 ≤ 20 ms, 주입→0 ≤ 스캔 주기 + 20 ms, '
                  '버튼→0 ≤ 20 ms; 정지 시점 여유 ≥ 0.25 m, 충돌 0; 버튼 래치는 reset_estop 성공 전까지 '
                  '(해제 false 뒤에도) 유지',
        log_format='estop_latency.csv [trial, kind, t_trigger, t_zero, latency_ms, '
                   'clearance_m] (음수 = 프로브 해상도, 그대로) + approach.csv',
        backends=(KINEMATIC, GAZEBO),
        needs=('amr_description', 'amr_localization', 'amr_navigation', 'amr_perception')),
    Scenario(
        number=10, slug='docking', title='정밀 도킹',
        spec='4.8 도킹 (위치 2 cm / 각도 1° 이내, 3회 실패 시 에러 보고)',
        metric='dock 액션 10회: GT 최종 자세 vs 기대 도킹 자세(월드 마커 판 면 + behavior.yaml standoff, '
               '마커 법선 반대 방위) — 서버 자기 보고는 참고; 마커 가림 1회: 재시도 소진',
        threshold='GT 위치 ≤ 0.02 m, 각도 ≤ 1°, 10/10; 마커 가림 시 success=false · attempts_used = 3; '
                  f'{REGISTRATION}',
        log_format='docking.csv [trial, dock_id, success, attempts, gt_pos_err_m, gt_ang_err_deg, '
                   'srv_pos_err_m, srv_ang_err_deg, time_s] (시행마다 갱신)',
        backends=(GAZEBO,), profiles=(SYSTEM,), needs=FULL),
    Scenario(
        number=11, slug='bt_recovery', title='BT 에러 복구',
        spec='4.8 행동 트리 (3가지 이상 에러 상황 자동 복구)',
        metric='작업 중 에러 주입 3종 — 경로 차단(작업 끝까지 유지), 위치 상실(순간 이동), 인식 실패(적재 도크 '
               '상자 치움, 인식 복구 관측 후 복원) → 복구 증거(우회 경로·RECOVERING·재위치추정·재인식) + task_status',
        threshold='3종 모두 주입 뒤 ≤ 60 s 에 복구 증거, 작업 COMPLETED (작업 ≤ 600 s)',
        log_format='recovery.csv [case, injected_at, recovered_at, recovery_s, final_status, '
                   'recovery_seen, phases, reset_before]',
        backends=(GAZEBO,), profiles=(SYSTEM,), needs=FULL),
    Scenario(
        number=12, slug='multi_robot_deadlock', title='5대 동시 운용 교착',
        spec='4.9 교통 관리 (교착 탐지·해소), 4.10 CPU ≤ 80 % (5대)',
        metric='(a) 도크 간 작업 → /fleet/task_events 완료, /fleet/traffic_events 탐지·해소, CPU = 시뮬레이터 포함 '
               '시스템 프로세스 CPU 합 / 호스트 코어 (b) 강제 교착: 교차로 x_ab_4 안 E-stop 로봇 + 실행기 작업으로 '
               '그 교차로를 지나야 하는 로봇 → BLOCKED 탐지·대체 경로 해소',
        threshold='(a) 로봇 5대, 작업 모두 COMPLETED (작업당 ≤ 600 s), 탐지된 교착은 모두 해소(UNRESOLVED 0), '
                  '평균 CPU ≤ 80 % '
                  '(b) traffic/DEADLOCK ≥ 1 이고 모두 traffic/RESOLVED',
        log_format='traffic.csv [recv_time, level, event, robot, message] + tasks.csv + cpu.csv '
                   '[time, system_cpu_percent, host_cpu_percent, cores_used] + forced.csv',
        backends=(GAZEBO,), profiles=(SYSTEM,), needs=FULL + ('amr_fleet',)),
    Scenario(
        number=13, slug='response_time', title='응답 시간',
        spec='4.10 성능 (명령 수신 ~ 로봇 반응 평균 200 ms, 50회 이상 평균/최대)',
        metric='작업 명령(assign_task, header.stamp = 발행 시각) → 로봇 첫 움직임 = ground_truth/odom 의 '
               '|v| ≥ 0.01 m/s 또는 |ω| ≥ 0.02 rad/s 첫 표본 (참고: |v| ≥ 0.05 m/s 변형). system: 실제 '
               'task_executor → Nav2 → velocity_profiler → safety_node / component: 실행기 대역 '
               '(체인 회귀용)',
        threshold='평균 ≤ 200 ms (벽시계, 최대 기록), 샘플 ≥ 50, 무응답 0',
        log_format='response_time.csv [cmd_time, response_time, latency_ms, cmd_id] '
                   '(amr_evaluation response_time_logger, odom 모드) + harness_response_time.csv '
                   '(+ wall_ms, latency_05_ms, cmd_vel_ms, rtf)',
        backends=(GAZEBO, KINEMATIC), profiles=(SYSTEM, COMPONENT),
        needs=FULL + ('amr_evaluation',)),
    Scenario(
        number=14, slug='soak', title='4시간 연속 운용',
        spec='4.10 안정성 (연속 4시간, 크래시·메모리 누수 없음)',
        metric='system 스택에 작업을 반복 투입(작업당 상한 시간)하며 프로세스별 RSS 샘플링(60 s), 선형 회귀 기울기, '
               '프로세스 종료 감시, 완료 작업 수',
        threshold='크래시 0, 프로세스별 RSS 증가율 ≤ 5 MB/h (첫 10 분 워밍업 제외, 적합 증가량 ≤ 2 MB 는 할당기 '
                  '요동으로 봄, 표본 < 5 는 판정 불가 = 실패), 작업 실패율 ≤ 2 % (작업당 900 s 초과 = 실패), '
                  '완료 작업 ≥ 4 /h × 기간 (멈춘 실행기 검출)',
        log_format='memory.csv [time, process, pid, rss_mb] + tasks.csv + soak.json',
        backends=(GAZEBO,), profiles=(SYSTEM,), long_running=True, needs=FULL),
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
    """tests/integration/README.md 의 시나리오 표 (지표·기준·로그 포맷·구성)."""
    def cell(text: str) -> str:
        return text.replace('|', '\\|')

    lines = ['| # | 시나리오 | 지표 | 합격 기준 | 로그 | 구성 / 백엔드 |',
             '| --- | --- | --- | --- | --- | --- |']
    for s in SCENARIOS:
        state = f'{", ".join(s.profiles)} / {", ".join(s.backends)}'
        if not s.implemented:
            state += ', 스켈레톤'
        if s.long_running:
            state += ', 장시간 (명시 선택 시만)'
        lines.append(f'| {s.number} | {s.title} | {cell(s.metric)} | {cell(s.threshold)} | '
                     f'`{s.log_format}` | {state} |')
    return '\n'.join(lines) + '\n'


__all__ = ['COMPONENT', 'SCENARIOS', 'by_id', 'get', 'table_markdown']
