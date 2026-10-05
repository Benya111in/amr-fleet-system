"""
매핑 행 정의 (rclpy 비의존) — 재생 구동기(replay_row.sh)·측정 노드(slam_probe.py)·시험이 같이 쓴다.

행마다: 이미지, 재생할 bag 토픽, 입력(누락 판정 대상), 처리 출력(대응 판정), 지도 토픽, 판정 패턴.
보고서 「3D LiDAR SLAM 도입 방안」 5단계 행: RTAB-Map 3D(구성 B), RTAB-Map 2D(같은 설정, scan 입력),
slam_toolbox(동기), LIO-SAM(madgwick 방위, 루프 on/off).

재생 토픽 규칙
- /clock 은 **bag 에 기록된 것**을 튼다 (시뮬레이션 시각). `ros2 bag play --clock` 은 쓰지 않는다 — Humble 기록기는
  메시지 시각을 수신 벽시계로 남기므로 --clock 이 내는 시각은 기록 당시 벽시계(1.79e9 s)이고, header 는 sim 시각
  (수 초~수백 초)이라 TF 조회가 전부 어긋난다 [측정: s3_pilot_a 첫 재생 row.log "earliest data is at time
  1791240775"]. 시간원은 기록된 /clock 하나다 (보고서 '재생 통제' 절의 "시간원 둘" 금지와 같은 뜻).
- 정답(ground_truth/odom)·원시 IMU(imu/data_raw)·imu/data 는 SLAM 행에 틀지 않는다 — 정답 방위·자세 누출 차단.
- LIO-SAM 행은 /tf 를 틀지 않는다 — LIO-SAM 이 liosam_odom → base_footprint 를 내므로 EKF 의
  odom → base_footprint 와 같이 틀면 base_footprint 의 부모가 둘이 된다 (tftree.check 가 잡는다).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

ROBOT = 'amr_01'


@dataclass(frozen=True)
class Row:
    """매핑 행 한 개."""

    name: str
    image: str
    launch: str                         # 컨테이너 안 ros2 launch 인자 (패키지 또는 경로 + 인자)
    play_topics: Tuple[str, ...]
    input_topic: str
    input_kind: str                     # 'cloud' | 'scan'
    map_topic: str = ''                 # OccupancyGrid (없으면 '')
    processed: Tuple[Tuple[str, str, str], ...] = ()
    # (토픽, 형 'odom' | 'cloud', QoS 'reliable' | 'best_effort') — 발행자 QoS 에 맞춘다: best-effort 발행자에
    # reliable 구독은 붙지 않는다 [측정: liosam_loop 첫 재생 probe.log "incompatible QoS"]
    anchors: Tuple[Tuple[str, str, str], ...] = (('map', 'slam_map', ''),)
    # (부모, 자식, via) — via '' = 스폰 자세 그대로, 'a>b' = 스폰 ⊕ TF(a→b) (LIO-SAM 오도메트리 원점 = LiDAR)
    fatal_patterns: Tuple[str, ...] = ()
    note_patterns: Tuple[str, ...] = ()
    # 노드가 처리한 프레임마다 한 줄 찍는 로그 문구 — 처리 수의 1 차 근거. 출력 토픽을 받아 세면 발행자 큐
    # (RTAB-Map odom 은 reliable keep_last 1)가 부하 때 덮여 구독자마다 다른 수가 나온다 [측정: rtabmap3d
    # v2 재생 — 로그 1 785 줄, 측정 노드 1 607 개, C++ 기록기 1 586 개]
    log_processed: str = ''
    extra: Dict[str, str] = field(default_factory=dict)


_R = f'/{ROBOT}'
CLOUD = f'{_R}/lidar3d/points'
SCAN = f'{_R}/scan_filtered'
IMU_MADGWICK = f'{_R}/imu/data_madgwick'
TF = ('/tf', '/tf_static')
CLOCK = ('/clock',)
#: 모든 행에서 판정에 쓰는 공통 경고 (재생·TF 문제)
COMMON_PATTERNS = ('Lookup would require extrapolation', 'TF_OLD_DATA', 'Message Filter dropping',
                   'queue is full', 'Did not receive data since')
#: LIO-SAM 처리 출력 — imageProjection 이 처리한 스캔마다 deskew/cloud_deskewed (time 이 없어 실제 보정은 없다),
#: mapOptimization 은 mappingProcessInterval(0.15 s) 마다 mapping/odometry.
#: 상류 QoS (utility.hpp): 점군 구독
#: best-effort 깊이 5, cloud_info 구독·mapping/odometry 발행 best-effort 깊이 1 — 매핑 단계는 바쁘면 버린다(설계)
LIOSAM_PROCESSED = (('/lio_sam/deskew/cloud_deskewed', 'cloud', 'reliable'),
                    ('/lio_sam/mapping/odometry', 'odom', 'best_effort'))
#: LIO-SAM 오도메트리 원점 = 출발 때 LiDAR 자세 → slam_map 아래에 장착 자리(URDF)만큼 옮겨 붙인다
LIOSAM_ANCHORS = (('map', 'slam_map', ''),
                  ('slam_map', f'{ROBOT}/liosam_odom',
                   f'{ROBOT}/base_footprint>{ROBOT}/lidar3d_link'))
#: icp_odometry 가 처리한 스캔마다 찍는 줄 (ratio=0 이면 정합 실패 null odom)
RTABMAP_PROCESSED = 'Odom: ratio='
#: IMU 동기 때문에 버린 스캔 — "We didn't receive IMU newer than previous image/scan … is dropped!"
RTABMAP_NOTE = ("didn't receive IMU newer", 'Odom: ratio=0.000000')
RTABMAP_FATAL = ('Failed to deskew', 'aborting odometry update', 'InvalidParameterTypeException',
                 'Odometry lost')

ROWS: Dict[str, Row] = {
    'rtabmap3d': Row(
        name='rtabmap3d', image='amr-ai/slam:0.1',
        launch='amr_slam3d rtabmap.launch.py dim:=3d',
        play_topics=CLOCK + TF + (CLOUD, IMU_MADGWICK), input_topic=CLOUD, input_kind='cloud',
        map_topic=f'{_R}/map', processed=((f'{_R}/icp_odom', 'odom', 'reliable'),),
        fatal_patterns=RTABMAP_FATAL, note_patterns=RTABMAP_NOTE,
        log_processed=RTABMAP_PROCESSED),
    'rtabmap2d': Row(
        name='rtabmap2d', image='amr-ai/slam:0.1',
        launch='amr_slam3d rtabmap.launch.py dim:=2d',
        play_topics=CLOCK + TF + (SCAN, IMU_MADGWICK), input_topic=SCAN, input_kind='scan',
        map_topic=f'{_R}/map', processed=((f'{_R}/icp_odom', 'odom', 'reliable'),),
        fatal_patterns=RTABMAP_FATAL, note_patterns=RTABMAP_NOTE,
        log_processed=RTABMAP_PROCESSED),
    'slam_toolbox': Row(
        name='slam_toolbox', image='amr-ai/slam:0.1',
        launch='amr_slam3d slam_toolbox.launch.py',
        play_topics=CLOCK + TF + (SCAN,), input_topic=SCAN, input_kind='scan',
        map_topic=f'{_R}/slam_toolbox/map', processed=(),
        fatal_patterns=('Message Filter dropping',)),
    'liosam_loop': Row(
        name='liosam_loop', image='amr-ai/liosam:0.1',
        launch='/ros2_ws/src/amr_slam3d/launch/liosam.launch.py loop:=true',
        play_topics=CLOCK + ('/tf_static', CLOUD, IMU_MADGWICK), input_topic=CLOUD,
        input_kind='cloud',
        processed=LIOSAM_PROCESSED,
        anchors=LIOSAM_ANCHORS,
        fatal_patterns=('ring channel not available', 'Invalid quaternion', 'queue is full'),
        note_patterns=('Point cloud timestamp not available', 'Waiting for IMU data')),
    'liosam_noloop': Row(
        name='liosam_noloop', image='amr-ai/liosam:0.1',
        launch='/ros2_ws/src/amr_slam3d/launch/liosam.launch.py loop:=false',
        play_topics=CLOCK + ('/tf_static', CLOUD, IMU_MADGWICK), input_topic=CLOUD,
        input_kind='cloud',
        processed=LIOSAM_PROCESSED,
        anchors=LIOSAM_ANCHORS,
        fatal_patterns=('ring channel not available', 'Invalid quaternion', 'queue is full'),
        note_patterns=('Point cloud timestamp not available', 'Waiting for IMU data')),
}

#: 파일럿 bag 에 기록할 토픽 (보고서 4단계: 2D 스캔·3D 점군·원시 IMU·휠·EKF odom·GT odom·TF + 재추정 IMU)
RECORD_TOPICS: Tuple[str, ...] = (
    '/clock', '/tf', '/tf_static',
    f'{_R}/scan', SCAN,
    f'{_R}/lidar3d/points_raw', CLOUD,
    f'{_R}/imu/data_raw', f'{_R}/imu/data', IMU_MADGWICK,
    f'{_R}/joint_states', f'{_R}/wheel_odom', f'{_R}/odometry/filtered',
    f'{_R}/ground_truth/odom', f'{_R}/cmd_vel',
)

#: 정답 정보를 담아 SLAM 행에 틀면 안 되는 토픽
ORACLE_TOPICS: Tuple[str, ...] = (f'{_R}/ground_truth/odom', f'{_R}/imu/data_raw',
                                  f'{_R}/imu/data')


def row(name: str) -> Row:
    """이름 → 행 (모르면 KeyError 에 가능한 이름을 붙인다)."""
    try:
        return ROWS[name]
    except KeyError:
        raise KeyError(f'모르는 행 {name!r} (가능: {", ".join(ROWS)})') from None


def scan_log(text: str, r: Row) -> Dict[str, Dict[str, int]]:
    """로그 본문에서 행의 치명·참고·공통 패턴 출현 수."""
    def count(pats: List[str]) -> Dict[str, int]:
        return {p: text.count(p) for p in pats}
    out = {'fatal': count(list(r.fatal_patterns)), 'note': count(list(r.note_patterns)),
           'common': count([p for p in COMMON_PATTERNS if p not in r.fatal_patterns])}
    if r.log_processed:
        out['processed_log_lines'] = text.count(r.log_processed)
    return out
