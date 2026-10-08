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
from typing import Dict, List, Sequence, Tuple

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
    # 행 런치 전에 같은 컨테이너에서 띄울 런치 (예: madgwick 을 재생 중에 다시 돌리는 변형 행)
    pre_launch: Tuple[str, ...] = ()
    # 재생할 bag 종류 — 'orig' = 파일럿 bag 그대로, 'madg' = madgwick 을 잠금 단계로 다시 계산해 넣은 파생 bag
    # (madg_lockstep.py). 파생 토픽(DERIVED_TOPICS)을 트는 행은 'madg' 여야 한다 (replay_row.sh 가 bag 토픽을 검사)
    bag: str = 'orig'
    # 정답 방위를 SLAM 노드가 **직접** 구독하는 오라클 행 — 상한 참고용으로만 표기하고 순위·비교에 쓰지 않는다
    oracle: bool = False
    # 입력의 마지막 n 장은 구조적으로 처리되지 않는다 (LIO-SAM imageProjection 2 장 캐시) → 누락 분모에서 뺀다
    tail_exempt: int = 0
    extra: Dict[str, str] = field(default_factory=dict)
    # ---- b2-eval (E3(b) 위치추정·E4 절제) ----
    # 위치추정 지도 종류: '' = 매핑 행, 'grid' = AMCL 이 읽을 map_server YAML (환경 LOC_MAP),
    # 'db' = RTAB-Map 위치추정 모드가 읽을 DB (환경 LOC_DB — 재생마다 출력 디렉터리로 복사해 쓴다)
    loc_map: str = ''
    # 재생 시작 전 initialpose 를 낸다 (bag 스폰을 위치추정 지도 프레임으로 — RTAB-Map 위치추정)
    initialpose: bool = False
    # 정답 지도 생성 행: ground_truth/odom 을 틀어 그 자세로 RTAB-Map DB 를 만든다 (SLAM 행이 아니다 — 오라클 입력)
    gt_builder: bool = False


_R = f'/{ROBOT}'
CLOUD = f'{_R}/lidar3d/points'
SCAN = f'{_R}/scan_filtered'
IMU_MADGWICK = f'{_R}/imu/data_madgwick'
#: 파생 bag(madg_lockstep.py)의 재계산 madgwick — 기록 때 생긴 0.14 s 공백이 없다. gain 0.1 (기본) / 0.01
IMU_MADG_RE = f'{_R}/imu/data_madgwick_re'
IMU_MADG_RE_G001 = f'{_R}/imu/data_madgwick_re_g001'
#: 파생 bag 의 오라클 IMU — imu/data(바이어스 보정 각속도·가속도) + 같은 스탬프 imu/data_raw 의 **정답 방위**.
#: imu/data 자체의 방위는 정답이 아니라 항등이다 (imu_filter_node.cpp 147–149행, 파일럿 bag yaw 가 끝까지 0 [측정]
#: b2-offline derive/yawcheck) — 정답 방위는 Gazebo 원시 imu/data_raw 에만 있다. 오라클 행만 SLAM 입력으로 쓴다
IMU_ORACLE = f'{_R}/imu/data_gtorient'
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
#: LIO-SAM imageProjection 은 점군을 2 장 쌓아 두고 새 장이 올 때 맨 앞을 처리한다 (08af3f3
#: imageProjection.cpp 228–230행 `if (cloudQueue.size() <= 2) return false;`) → bag 의 마지막 2 장은
#: 어떤 기록에서도
#: 처리되지 않는다 (rtabmap.md §4.1 리뷰 B1)
LIOSAM_TAIL = 2
#: icp_odometry 가 처리한 스캔마다 찍는 줄 (ratio=0 이면 정합 실패 null odom)
RTABMAP_PROCESSED = 'Odom: ratio='
#: IMU 동기 때문에 버린 스캔 — "We didn't receive IMU newer than previous image/scan … is dropped!"
RTABMAP_NOTE = ("didn't receive IMU newer", 'Odom: ratio=0.000000')
RTABMAP_FATAL = ('Failed to deskew', 'aborting odometry update', 'InvalidParameterTypeException',
                 'Odometry lost')

LIOSAM_FATAL = ('ring channel not available', 'Invalid quaternion', 'queue is full')
LIOSAM_NOTE = ('Point cloud timestamp not available', 'Waiting for IMU data',
               'Large velocity, reset IMU-preintegration', 'Large bias, reset IMU-preintegration')
LIOSAM_LAUNCH = '/ros2_ws/src/amr_slam3d/launch/liosam.launch.py'


def _liosam(name: str, args: str, imu: str, oracle: bool = False) -> Row:
    """파생 bag 에서 도는 LIO-SAM 변형 행 — IMU 토픽과 덮어쓸 파라미터만 다르다."""
    flag = ' oracle:=true' if oracle else ''
    return Row(name=name, image='amr-ai/liosam:0.1',
               launch=f'{LIOSAM_LAUNCH} {args} imu_topic:={imu}{flag}',
               play_topics=CLOCK + ('/tf_static', CLOUD, imu), input_topic=CLOUD,
               input_kind='cloud', processed=LIOSAM_PROCESSED, anchors=LIOSAM_ANCHORS,
               fatal_patterns=LIOSAM_FATAL, note_patterns=LIOSAM_NOTE, bag='madg', oracle=oracle,
               tail_exempt=LIOSAM_TAIL)


def _rtabmap(name: str, dim: str, imu: str, bag: str = 'orig') -> Row:
    """RTAB-Map 행 — imu '' 이면 IMU 를 구독하지 않는다 (wait_imu_to_init false, 리서치 R11)."""
    inp = CLOUD if dim == '3d' else SCAN
    return Row(name=name, image='amr-ai/slam:0.1',
               launch=f'amr_slam3d rtabmap.launch.py dim:={dim} imu_topic:={imu or "none"}',
               play_topics=CLOCK + TF + ((inp, imu) if imu else (inp,)), input_topic=inp,
               input_kind='cloud' if dim == '3d' else 'scan', map_topic=f'{_R}/map',
               processed=((f'{_R}/icp_odom', 'odom', 'reliable'),),
               fatal_patterns=RTABMAP_FATAL, note_patterns=RTABMAP_NOTE,
               log_processed=RTABMAP_PROCESSED, bag=bag)


GT_ODOM = f'{_R}/ground_truth/odom'
#: 컨테이너 안에서 만들어지는 토픽 (재생하지 않는다) — 구독 확인 대상이 될 수 있다
MAP_AMCL = f'{_R}/map_amcl'
INITIALPOSE = f'{_R}/initialpose'
INTERNAL_TOPICS: Tuple[str, ...] = (MAP_AMCL, INITIALPOSE)
#: E4 절제 실험 파라미터 파일 (한 손잡이만) — 작업공간 소스 경로 (재생 컨테이너가 /ros2_ws 로 마운트)
ABLATION_DIR = '/ros2_ws/src/amr_slam3d/config/ablation'
CLOUD_RAW = f'{_R}/lidar3d/points_raw'
AMCL_LAUNCH = 'amr_slam3d amcl_loc.launch.py'


def _ablation(name: str, knob: str = '', cloud: str = CLOUD) -> Row:
    """RTAB-Map 3D (IMU 없음, 휠 guess) 에서 한 손잡이만 바꾼 행 — 파라미터 파일 또는 입력 점군."""
    base = _rtabmap(name, '3d', '')
    launch = base.launch + (f' extra_params:={ABLATION_DIR}/{knob}.yaml' if knob else '')
    if cloud != CLOUD:
        launch += f' cloud_topic:={cloud}'
    return Row(name=name, image=base.image, launch=launch,
               play_topics=CLOCK + TF + (cloud,), input_topic=cloud, input_kind='cloud',
               map_topic=base.map_topic, processed=base.processed,
               fatal_patterns=base.fatal_patterns, note_patterns=base.note_patterns,
               log_processed=base.log_processed, bag='orig')


def _amcl(name: str) -> Row:
    """AMCL 위치추정 (배포 amcl.yaml) — 지도는 재생마다 LOC_MAP 으로 준다. AMCL 의 global frame = map(월드)."""
    return Row(name=name, image='amr-ai/slam:0.1', launch=AMCL_LAUNCH,
               play_topics=CLOCK + TF + (SCAN,), input_topic=SCAN, input_kind='scan',
               anchors=(), fatal_patterns=(), note_patterns=('Message Filter dropping',),
               loc_map='grid')


def _rtabloc(name: str) -> Row:
    """RTAB-Map 3D 위치추정 모드 (구성 B 오도메트리 + 지도 DB, 증분 끔) — DB 는 재생마다 LOC_DB 로 준다."""
    base = _rtabmap(name, '3d', '')
    return Row(name=name, image=base.image, launch=base.launch + ' localization:=true',
               play_topics=base.play_topics, input_topic=base.input_topic, input_kind='cloud',
               map_topic=base.map_topic, processed=base.processed,
               fatal_patterns=base.fatal_patterns, note_patterns=base.note_patterns,
               log_processed=base.log_processed, loc_map='db', initialpose=True)


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
    # 변형: 기록된 imu/data_madgwick 대신 재생 중 imu/data(바이어스 보정, 공백 없음)에서 madgwick 을 다시 돌린다.
    # 파일럿 bag 의 madgwick 출력에 0.14 s 공백(sim 9.48 s)이 있어 IMU 를 쓰는 행이 그 스캔을 버린 것을 가르기 위한 행
    'rtabmap3d_imulive': Row(
        name='rtabmap3d_imulive', image='amr-ai/slam:0.1',
        launch='amr_slam3d rtabmap.launch.py dim:=3d',
        play_topics=CLOCK + TF + (CLOUD, f'{_R}/imu/data'), input_topic=CLOUD, input_kind='cloud',
        map_topic=f'{_R}/map', processed=((f'{_R}/icp_odom', 'odom', 'reliable'),),
        fatal_patterns=RTABMAP_FATAL, note_patterns=RTABMAP_NOTE,
        log_processed=RTABMAP_PROCESSED,
        pre_launch=('amr_slam3d madgwick.launch.py robot:=amr_01',)),
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
        fatal_patterns=LIOSAM_FATAL,
        note_patterns=LIOSAM_NOTE, tail_exempt=LIOSAM_TAIL),
    'liosam_noloop': Row(
        name='liosam_noloop', image='amr-ai/liosam:0.1',
        launch='/ros2_ws/src/amr_slam3d/launch/liosam.launch.py loop:=false',
        play_topics=CLOCK + ('/tf_static', CLOUD, IMU_MADGWICK), input_topic=CLOUD,
        input_kind='cloud',
        processed=LIOSAM_PROCESSED,
        anchors=LIOSAM_ANCHORS,
        fatal_patterns=LIOSAM_FATAL,
        note_patterns=LIOSAM_NOTE, tail_exempt=LIOSAM_TAIL),
    # ---- b2-offline (2026-10-06): 파생 bag 의 재계산 IMU, IMU 제거, LIO-SAM 평면·절제 행 ----
    'rtabmap3d_noimu': _rtabmap('rtabmap3d_noimu', '3d', ''),
    'rtabmap2d_noimu': _rtabmap('rtabmap2d_noimu', '2d', ''),
    'rtabmap3d_imu': _rtabmap('rtabmap3d_imu', '3d', IMU_MADG_RE, bag='madg'),
    'liosam_loop_re': _liosam('liosam_loop_re', 'loop:=true', IMU_MADG_RE),
    'liosam_noloop_re': _liosam('liosam_noloop_re', 'loop:=false', IMU_MADG_RE),
    'liosam_rpy0': _liosam('liosam_rpy0', 'loop:=true rpy_weight:=0.0', IMU_MADG_RE),
    'liosam_g001': _liosam('liosam_g001', 'loop:=true', IMU_MADG_RE_G001),
    'liosam_planar': _liosam('liosam_planar',
                             'loop:=true rpy_weight:=0.0 z_tol:=0.0 rot_tol:=0.0', IMU_MADG_RE),
    'liosam_oracle': _liosam('liosam_oracle', 'loop:=true', IMU_ORACLE, oracle=True),
    # ---- b2-eval (2026-10-09): E3(b) 위치추정 모드, 정답 자세 DB, E4 절제 ----
    'amcl_gt': _amcl('amcl_gt'),
    'amcl_slam': _amcl('amcl_slam'),
    'rtabmap3d_loc_gt': _rtabloc('rtabmap3d_loc_gt'),
    'rtabmap3d_loc_slam': _rtabloc('rtabmap3d_loc_slam'),
    # 정답 지도(RTAB-Map 표현): 같은 장들을 정답 자세(ground_truth/odom → gt_odom TF)에 놓은 DB. 루프·근접 끔
    'rtabmap3d_gtdb': Row(
        name='rtabmap3d_gtdb', image='amr-ai/slam:0.1',
        launch='amr_slam3d rtabmap.launch.py dim:=3d imu_topic:=none odom_mode:=gt_tf',
        play_topics=CLOCK + ('/tf_static', CLOUD, GT_ODOM), input_topic=CLOUD,
        input_kind='cloud', map_topic=f'{_R}/map', fatal_patterns=RTABMAP_FATAL,
        gt_builder=True),
    'rtabmap3d_abl_vox005': _ablation('rtabmap3d_abl_vox005', 'vox005'),
    'rtabmap3d_abl_vox02': _ablation('rtabmap3d_abl_vox02', 'vox02'),
    'rtabmap3d_abl_vox03': _ablation('rtabmap3d_abl_vox03', 'vox03'),
    'rtabmap3d_abl_p2p': _ablation('rtabmap3d_abl_p2p', 'p2p'),
    'rtabmap3d_abl_obs10': _ablation('rtabmap3d_abl_obs10', 'obs10'),
    'rtabmap3d_abl_obs20': _ablation('rtabmap3d_abl_obs20', 'obs20'),
    'rtabmap3d_abl_opt10': _ablation('rtabmap3d_abl_opt10', 'opt10'),
    'rtabmap3d_abl_opt30': _ablation('rtabmap3d_abl_opt30', 'opt30'),
    'rtabmap3d_abl_raw': _ablation('rtabmap3d_abl_raw', '', CLOUD_RAW),
}

#: 파생 bag 에만 있는 토픽 (madg_lockstep.py 가 만든다)
DERIVED_TOPICS: Tuple[str, ...] = (IMU_MADG_RE, IMU_MADG_RE_G001, IMU_ORACLE)

#: 시나리오 bag 에 기록할 토픽 (보고서 4단계: 2D 스캔·3D 점군·원시 IMU·휠·EKF odom·GT odom·TF).
#: **madgwick 출력은 기록하지 않는다** (b2-rec, replay.md §9-1): 기록 때 madgwick 이 best-effort 입력을 잃어 필터 상태가
#: 달라졌다 (replay.md §1.2). madgwick 은 녹화 뒤 madg_lockstep.py 로 imu/data 에서 결정적으로 파생한다
#: (DERIVED_TOPICS)
RECORD_TOPICS: Tuple[str, ...] = (
    '/clock', '/tf', '/tf_static',
    f'{_R}/scan', SCAN,
    f'{_R}/lidar3d/points_raw', CLOUD,
    f'{_R}/imu/data_raw', f'{_R}/imu/data',
    f'{_R}/joint_states', f'{_R}/wheel_odom', f'{_R}/odometry/filtered',
    f'{_R}/ground_truth/odom', f'{_R}/cmd_vel',
)
#: 파일럿 bag(s3_pilot_a)에만 있는 토픽 — 기록된 madgwick. 이것을 트는 옛 행(rtabmap3d·rtabmap2d·liosam_loop·
#: liosam_noloop)은 파일럿 bag 재현용이고, 새 bag 에서는 replay_row.sh 의 토픽 사전 검사가 재생 전에 멈춘다
PILOT_ONLY_TOPICS: Tuple[str, ...] = (IMU_MADGWICK,)

#: SLAM 행에 틀면 안 되는 토픽. ground_truth/odom·imu/data_raw·data_gtorient 는 정답을 담는다.
#: imu/data 는 방위가 항등(정답 아님)이지만 roll·pitch 0 이 이 평평한 월드의 정답과 같아 평면 사전 노릇을 할 수 있어
#: 함께 막는다 (madgwick 입력으로만 튼다)
ORACLE_TOPICS: Tuple[str, ...] = (f'{_R}/ground_truth/odom', f'{_R}/imu/data_raw',
                                  f'{_R}/imu/data', IMU_ORACLE)


def row(name: str) -> Row:
    """이름 → 행 (모르면 KeyError 에 가능한 이름을 붙인다)."""
    try:
        return ROWS[name]
    except KeyError:
        raise KeyError(f'모르는 행 {name!r} (가능: {", ".join(ROWS)})') from None


def bag_topics(metadata_text: str) -> List[str]:
    """rosbag2 metadata.yaml 본문 → 토픽 이름 목록."""
    import yaml
    info = yaml.safe_load(metadata_text)['rosbag2_bagfile_information']
    return [t['topic_metadata']['name'] for t in info.get('topics_with_message_count', [])]


def missing_play_topics(r: Row, topics: List[str]) -> List[str]:
    """
    행이 틀 토픽 중 bag 에 없는 것 — 재생 전에 멈추는 근거.

    파생 bag 토픽(재계산 IMU)을 원 bag 으로 재생하면 IMU 없이 돌게 된다 — 재생기가 없는 토픽을 어떻게 다루는지
    (오류인지 무시인지)에 기대지 않고 재생 전에 멈춘다.
    """
    have = set(topics)
    return [t for t in r.play_topics if t not in have]


def scan_log(text: str, r: Row) -> Dict[str, Dict[str, int]]:
    """로그 본문에서 행의 치명·참고·공통 패턴 출현 수."""
    def count(pats: List[str]) -> Dict[str, int]:
        return {p: text.count(p) for p in pats}
    out = {'fatal': count(list(r.fatal_patterns)), 'note': count(list(r.note_patterns)),
           'common': count([p for p in COMMON_PATTERNS if p not in r.fatal_patterns])}
    if r.log_processed:
        out['processed_log_lines'] = text.count(r.log_processed)
    return out


# ---- 재생 시작 전 구독 확인 (b2-rec, replay.md §9-4) ------------------------------------------------
# 고정 15 s 대기는 부하 때 모자라 LIO-SAM 이 첫 2–3 장을 "Waiting for IMU data" 로 버렸다
# (replay.md §5·§7, 43 재생 중 4).
# replay_row.sh 는 `ros2 bag play -p`(멈춤)로 재생기 발행자를 먼저 만들고, 아래 (토픽, 구독 노드) 가 모두
# `ros2 topic info -v` 에 보이고 재생기(rosbag2_player)가 그 토픽의 발행자로 보일 때 resume 서비스를 부른다.
PLAYER_NODE = 'rosbag2_player'
RECORDER_NODE = 'rosbag2_recorder'
PROBE_NODE = 'slam_probe'


def ready_requirements(r: Row) -> Tuple[Tuple[str, str], ...]:
    """행이 재생 전에 연결돼 있어야 하는 (토픽, 구독 노드 이름) 목록 (노드 이름 = Node name 줄, 네임스페이스 무시)."""
    imu = next((t for t in r.play_topics if '/imu/' in t), '')
    req = [(r.input_topic, PROBE_NODE)]
    if 'odom_mode:=gt_tf' in r.launch:
        req += [(r.input_topic, 'rtabmap'), (GT_ODOM, 'gt_odom_tf')]
    elif 'amcl_loc.launch.py' in r.launch:
        req += [(r.input_topic, 'amcl'), (MAP_AMCL, 'amcl')]
    elif 'rtabmap.launch.py' in r.launch:
        req += [(r.input_topic, 'icp_odometry'), (r.input_topic, 'rtabmap')]
        if imu and r.pre_launch:
            req += [(imu, 'imu_filter_madgwick'), (IMU_MADGWICK, 'icp_odometry')]
        elif imu:
            req += [(imu, 'icp_odometry')]
    elif 'slam_toolbox' in r.launch:
        req += [(r.input_topic, 'slam_toolbox')]
    elif 'liosam' in r.launch:
        req += [(r.input_topic, 'lio_sam_imageProjection'), (imu, 'lio_sam_imageProjection'),
                (imu, 'lio_sam_imu_preintegration')]
    else:
        raise ValueError(f'행 {r.name}: 구독 확인 규칙이 없다')
    if r.initialpose:
        req += [(INITIALPOSE, 'rtabmap')]
    # 처리 출력의 C++ 기록기 (replay_row.sh out_bag) — 첫 출력부터 세려면 재생 전에 붙어 있어야 한다
    req += [(t, RECORDER_NODE) for t, _, _ in r.processed]
    return tuple(req)


def parse_topic_info(text: str) -> Dict[str, List[str]]:
    """`ros2 topic info -v` 출력 → {'publishers': [노드 이름], 'subscribers': [노드 이름]} (Humble 형식)."""
    out: Dict[str, List[str]] = {'publishers': [], 'subscribers': []}
    name = None
    for line in text.splitlines():
        line = line.strip()
        if line.startswith('Node name:'):
            name = line.split(':', 1)[1].strip()
        elif line.startswith('Endpoint type:') and name is not None:
            kind = line.split(':', 1)[1].strip()
            if kind == 'PUBLISHER':
                out['publishers'].append(name)
            elif kind == 'SUBSCRIPTION':
                out['subscribers'].append(name)
            name = None
    return out


def unmet(reqs: Sequence[Tuple[str, str]], infos: Dict[str, str],
          player_topics: Sequence[str] = ()) -> List[str]:
    """
    아직 안 된 조건 목록 (비면 재생을 시작해도 된다).

    infos = {토픽: ros2 topic info -v 출력}. player_topics 의 토픽은 재생기 발행자도 보여야 한다.
    """
    miss = []
    parsed = {t: parse_topic_info(v) for t, v in infos.items()}
    for topic, node in reqs:
        if node not in parsed.get(topic, {}).get('subscribers', []):
            miss.append(f'{topic} ← {node}')
    for topic in player_topics:
        if PLAYER_NODE not in parsed.get(topic, {}).get('publishers', []):
            miss.append(f'{topic} → {PLAYER_NODE}')
    return miss


# ---- 녹화 기록기 구독 QoS (b2-rec 시운전 1 회차 뒤) ------------------------------------------
#: 기록기 구독 깊이 — 기본 10 이면 1 kHz /clock 은 기록기 실행기가 10 ms 만 밀려도 표본을 잃는다
#: (시운전 1 회차 sim 81 s 에 /clock 2 ms 간격 누락 5 회, joint_states 30 개 부족 [측정 dryrun/rec/dry_s6wall_a1]).
#: 발행자는 모두 reliable·volatile (같은 bag metadata.yaml) 이라 구독도 그것에 맞춘다 — 다르게 주면 짝이 안 맞아
#: 아무것도 기록되지 않는다.
#: /tf_static 은 transient_local 이라 덮어쓰지 않는다 (기록기가 발행자에 맞춘다).
RECORD_QOS_DEPTH: Dict[str, int] = {
    '/clock': 1000, '/tf': 1000, f'{_R}/joint_states': 1000, f'{_R}/imu/data_raw': 1000,
    f'{_R}/imu/data': 1000, f'{_R}/odometry/filtered': 500, f'{_R}/wheel_odom': 500,
    f'{_R}/ground_truth/odom': 500, f'{_R}/cmd_vel': 200, f'{_R}/scan': 100, SCAN: 100,
    f'{_R}/lidar3d/points_raw': 50, CLOUD: 50,
}


def record_qos_yaml() -> str:
    """`ros2 bag record --qos-profile-overrides-path` 파일 본문 (RECORD_QOS_DEPTH 에 있는 기록 토픽)."""
    out = []
    for t in RECORD_TOPICS:
        if t in RECORD_QOS_DEPTH:
            out.append(f'{t}:\n  history: keep_last\n  depth: {RECORD_QOS_DEPTH[t]}\n'
                       '  reliability: reliable\n  durability: volatile\n')
    return ''.join(out)
