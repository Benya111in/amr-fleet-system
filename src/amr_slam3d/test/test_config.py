"""행 설정·런치 계약 시험 — 문자열 파라미터, deskew 끔, 정답 방위 누출 차단, 재생 토픽 규칙, LIO-SAM 외부 파라미터."""

import importlib.util
from pathlib import Path
import re

from amr_slam3d import rows
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[1]


def _load_launch(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), PKG / 'launch' / name)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _yaml(name):
    return yaml.safe_load((PKG / 'config' / name).read_text(encoding='utf-8'))


def test_rtabmap_core_parameters_are_strings():
    """Grp/Name 형식(RTAB-Map 코어) 파라미터는 문자열 — bool·숫자면 노드가 abort (s0 VERSIONS.md 1.1)."""
    data = _yaml('rtabmap.yaml')
    n = 0
    for block in data.values():
        for key, value in block['ros__parameters'].items():
            if re.fullmatch(r'[A-Z][A-Za-z0-9]*/[A-Za-z0-9]+', key):
                assert isinstance(value, str), (key, value)
                n += 1
    assert n >= 40


def test_rtabmap_deskew_off_and_planar():
    data = _yaml('rtabmap.yaml')
    odom = data['/**/icp_odometry']['ros__parameters']
    shared = data['/**']['ros__parameters']
    assert odom['deskewing'] is False               # time 필드 없음 → 켜면 "Failed to deskew"
    assert shared['Reg/Force3DoF'] == 'true'
    assert shared['Icp/RangeMin'] == '0.2'
    # icp_odometry 가 옮겨 가는 두 값은 ROS 파라미터와 코어 값이 같아야 한다
    assert float(shared['Icp/VoxelSize']) == odom['scan_voxel_size']
    assert int(shared['Icp/PointToPlaneK']) == odom['scan_normal_k']


def test_rtabmap_launch_rejects_oracle_imu():
    mod = _load_launch('rtabmap.launch.py')
    for bad in ('/amr_01/imu/data_raw', '/amr_01/imu/data', 'imu/data'):
        with pytest.raises(RuntimeError):
            mod.check_imu_topic(bad)
    mod.check_imu_topic('/amr_01/imu/data_madgwick')
    mod.check_imu_topic('')


def test_rtabmap_launch_2d_3d_share_everything_but_input():
    mod = _load_launch('rtabmap.launch.py')
    a = mod.node_setup('3d', 'amr_01', '/c', '/s', '/amr_01/imu/data_madgwick', True, 'slam_map',
                       '/db', 'p.yaml', 'info')
    b = mod.node_setup('2d', 'amr_01', '/c', '/s', '/amr_01/imu/data_madgwick', True, 'slam_map',
                       '/db', 'p.yaml', 'info')
    assert a[0] == b[0]                              # icp_odometry 파라미터 동일
    diff = {k for k in a[1][1] if a[1][1][k] != b[1][1][k]}
    assert diff == {'subscribe_scan_cloud', 'subscribe_scan'}
    ra, rb = dict(a[2]), dict(b[2])
    assert ra['scan_cloud'] == '/c' and ra['scan'] == 'scan_not_used'
    assert rb['scan'] == '/s' and rb['scan_cloud'] == 'scan_cloud_not_used'
    assert a[0][1]['guess_frame_id'] == 'amr_01/odom'
    assert a[0][1]['odom_frame_id'] == 'amr_01/icp_odom'
    no_guess = mod.node_setup('3d', 'amr_01', '/c', '/s', '', False, 'slam_map', '/db', 'p', 'i')
    assert no_guess[0][1]['guess_frame_id'] == '' and no_guess[0][1]['wait_imu_to_init'] is False
    with pytest.raises(RuntimeError):
        mod.node_setup('4d', 'amr_01', '/c', '/s', '', True, 'm', '/db', 'p', 'i')


def test_madgwick_no_mag_no_tf():
    p = _yaml('madgwick.yaml')['/**/imu_filter_madgwick']['ros__parameters']
    assert p['use_mag'] is False and p['publish_tf'] is False
    assert p['remove_gravity_vector'] is False


def test_rows_play_recorded_clock_and_never_oracle_topics():
    """시간원 = 기록된 /clock (--clock 은 기록 벽시계를 낸다), 정답 토픽은 재생하지 않는다."""
    for r in rows.ROWS.values():
        assert '/clock' in r.play_topics
        for t in rows.ORACLE_TOPICS:
            if t.endswith('imu/data') and r.pre_launch:
                # imu/data 는 madgwick 입력으로만 튼다 — SLAM 노드는 madgwick 출력만 구독한다
                assert any('madgwick' in p for p in r.pre_launch), r.name
                continue
            if t == rows.IMU_ORACLE and r.oracle:
                # 오라클 행: SLAM 노드가 정답 방위를 직접 받는다 — 상한 참고용으로만 표기 (런치가 oracle:=true 요구)
                assert 'oracle:=true' in r.launch, r.name
                continue
            assert t not in r.play_topics, (r.name, t)
        assert '/tf_static' in r.play_topics
        assert r.input_topic in r.play_topics


def test_oracle_list_covers_known_ground_truth_topics():
    """정답 토픽 목록 자체를 고정 목록으로 단언 — 목록에서 빠진 토픽은 위 시험이 못 본다 (리뷰 M9)."""
    known = {'/amr_01/ground_truth/odom', '/amr_01/imu/data_raw', '/amr_01/imu/data',
             '/amr_01/imu/data_gtorient'}
    assert known <= set(rows.ORACLE_TOPICS)
    for r in rows.ROWS.values():
        # 정답 자세·원시 정답 방위는 madgwick 변형 행을 포함해 어떤 행도 틀지 않는다
        assert '/amr_01/ground_truth/odom' not in r.play_topics, r.name
        assert '/amr_01/imu/data_raw' not in r.play_topics, r.name
    # imu/data 를 트는 행은 madgwick 변형 행 하나뿐이다 (madgwick 입력으로만 — rtabmap.md §1.1)
    assert [n for n, r in rows.ROWS.items() if '/amr_01/imu/data' in r.play_topics] == \
        ['rtabmap3d_imulive']
    # 정답 방위 합성 토픽은 오라클 행만 튼다
    assert [n for n, r in rows.ROWS.items() if rows.IMU_ORACLE in r.play_topics] == \
        ['liosam_oracle']
    # 오라클 표시는 이름에도 드러나야 표에서 섞이지 않는다
    assert [n for n, r in rows.ROWS.items() if r.oracle] == ['liosam_oracle']
    for n, r in rows.ROWS.items():
        assert ('oracle:=true' in r.launch) == r.oracle, n


def test_liosam_rows_do_not_play_dynamic_tf():
    """LIO-SAM 은 odometryFrame → base_footprint 를 낸다 — EKF /tf 를 같이 틀면 부모가 둘."""
    for name, r in rows.ROWS.items():
        if name.startswith('liosam_'):
            assert '/tf' not in r.play_topics, name
        else:
            assert '/tf' in r.play_topics, name     # 휠-EKF guess 가 TF 로 들어온다


def test_record_topics_cover_report_list():
    """보고서 4단계: 2D 스캔·3D 점군(전처리 전)·원시 IMU·휠·EKF odom·GT odom·TF."""
    need = {'/amr_01/scan', '/amr_01/lidar3d/points_raw', '/amr_01/imu/data_raw',
            '/amr_01/wheel_odom', '/amr_01/odometry/filtered', '/amr_01/ground_truth/odom',
            '/tf', '/tf_static', '/clock'}
    assert need <= set(rows.RECORD_TOPICS)
    # b2-rec: 기록된 madgwick 은 새 bag 에 없다 — 파일럿 bag 재현용 옛 행 넷만 그것을 튼다
    assert rows.IMU_MADGWICK not in rows.RECORD_TOPICS
    assert {'/amr_01/imu/data_raw', '/amr_01/imu/data'} <= set(rows.RECORD_TOPICS)
    pilot_rows = {n for n, r in rows.ROWS.items()
                  if set(r.play_topics) & set(rows.PILOT_ONLY_TOPICS)}
    assert pilot_rows == {'rtabmap3d', 'rtabmap2d', 'liosam_loop', 'liosam_noloop'}
    for r in rows.ROWS.values():
        for t in r.play_topics:
            assert t in rows.RECORD_TOPICS + rows.DERIVED_TOPICS + rows.PILOT_ONLY_TOPICS, \
                (r.name, t)
        # 파생 토픽(재계산 madgwick)을 트는 행은 파생 bag 이 필요하다고 표시한다
        uses_derived = any(t in rows.DERIVED_TOPICS for t in r.play_topics)
        assert uses_derived == (r.bag == 'madg'), r.name
    assert not set(rows.DERIVED_TOPICS) & set(rows.RECORD_TOPICS)


def test_liosam_config_matches_sensor_contract_and_urdf():
    p = _yaml('liosam.yaml')['/**']['ros__parameters']
    assert p['sensor'] == 'velodyne' and p['N_SCAN'] == 16 and p['Horizon_SCAN'] == 1800
    assert p['imuTopic'].endswith('imu/data_madgwick')
    assert p['pointCloudTopic'] == rows.CLOUD
    assert p['baselinkFrame'] == 'amr_01/base_footprint'
    assert p['odometryFrame'] not in ('odom', 'amr_01/odom')     # EKF odom 과 겹치면 안 된다
    # extrinsicTrans = LiDAR 프레임에서 본 IMU 위치 = imu − lidar (URDF 의 두 joint 에서 계산)
    lidar = yaml.safe_load((REPO / 'src/amr_description/config/lidar3d.yaml')
                           .read_text(encoding='utf-8'))
    text = (REPO / 'src/amr_description/test/golden/fleet_amr_01.urdf').read_text(encoding='utf-8')
    imu_xyz = re.search(r'<joint name="amr_01/imu_joint".*?xyz="([^"]+)"', text, re.S).group(1)
    imu_z = float(imu_xyz.split()[2])               # base_link 기준
    mast = lidar['/**']['ros__parameters'] if '/**' in lidar else lidar
    assert mast is not None
    assert p['extrinsicTrans'] == pytest.approx([-0.27, 0.0, imu_z - 0.72])
    assert p['lidarMinRange'] == pytest.approx(0.2)


def test_liosam_launch_loop_flag():
    mod = _load_launch('liosam.launch.py')
    assert Path(mod.default_params()).name == 'liosam.yaml'
    assert Path(mod.default_params()).is_file()
    assert len(mod.EXECUTABLES) == 4


def test_scan_log_counts_patterns():
    r = rows.row('rtabmap3d')
    s = rows.scan_log('x Failed to deskew input cloud y\nFailed to deskew', r)
    assert s['fatal']['Failed to deskew'] == 2
    assert s['fatal']['aborting odometry update'] == 0


def test_scan_log_counts_processed_lines():
    r = rows.row('rtabmap2d')
    text = 'Odom: ratio=0.5 x\nOdom: ratio=0.000000 y\nWe didn\'t receive IMU newer than previous'
    s = rows.scan_log(text, r)
    assert s['processed_log_lines'] == 2
    assert s['note']['Odom: ratio=0.000000'] == 1 and s['note']["didn't receive IMU newer"] == 1
    assert 'processed_log_lines' not in rows.scan_log(text, rows.row('liosam_loop'))


def _launch_args(r):
    """행 런치 문자열의 k:=v 인자."""
    return dict(a.split(':=', 1) for a in r.launch.split() if ':=' in a)


def test_liosam_overrides_and_oracle_guard():
    mod = _load_launch('liosam.launch.py')
    assert mod.overrides('true') == {'loopClosureEnableFlag': True}
    assert mod.overrides('False') == {'loopClosureEnableFlag': False}
    for bad in ('/amr_01/imu/data', '/amr_01/imu/data_raw', 'imu/data', rows.IMU_ORACLE):
        with pytest.raises(RuntimeError):
            mod.overrides('true', imu_topic=bad)              # 오라클 표시 없이 정답(·항등) 방위
    assert mod.overrides('true', imu_topic=rows.IMU_ORACLE, oracle='true')['imuTopic'] == \
        rows.IMU_ORACLE
    with pytest.raises(RuntimeError):
        mod.overrides('true', imu_topic='/amr_01/imu/data_madgwick_re', oracle='true')
    with pytest.raises(RuntimeError):
        mod.overrides('true', oracle='true')                  # 설정 파일 IMU 는 오라클이 아니다
    with pytest.raises(RuntimeError):
        mod.overrides('maybe')
    with pytest.raises(RuntimeError):
        mod.overrides('true', oracle='yes')
    with pytest.raises(RuntimeError):
        mod.overrides('true', z_tol='-0.1')
    p = mod.overrides('true', rpy_weight='0.0', z_tol='0', rot_tol='0.0')
    assert p == {'loopClosureEnableFlag': True, 'imuRPYWeight': 0.0, 'z_tollerance': 0.0,
                 'rotation_tollerance': 0.0}


def test_liosam_rows_resolve_to_intended_parameters():
    """행 이름과 실제 덮어쓰기가 맞는지 — 평면 행만 z/rot 0, 절제 행은 한 가지씩, 구독 IMU 는 재생된다."""
    mod = _load_launch('liosam.launch.py')
    base = _yaml('liosam.yaml')['/**']['ros__parameters']
    want = {'liosam_loop_re': {}, 'liosam_noloop_re': {},
            'liosam_rpy0': {'imuRPYWeight': 0.0},
            'liosam_g001': {},
            'liosam_planar': {'imuRPYWeight': 0.0, 'z_tollerance': 0.0,
                              'rotation_tollerance': 0.0},
            'liosam_oracle': {}}
    imu = {'liosam_g001': rows.IMU_MADG_RE_G001, 'liosam_oracle': rows.IMU_ORACLE}
    for name, extra in want.items():
        r = rows.row(name)
        a = _launch_args(r)
        p = mod.overrides(a['loop'], a.get('imu_topic', ''), a.get('rpy_weight', ''),
                          a.get('z_tol', ''), a.get('rot_tol', ''), a.get('oracle', 'false'))
        assert p['imuTopic'] == imu.get(name, rows.IMU_MADG_RE), name
        assert p['imuTopic'] in r.play_topics, name
        assert p['loopClosureEnableFlag'] == (name != 'liosam_noloop_re'), name
        assert {k: v for k, v in p.items() if k not in ('imuTopic', 'loopClosureEnableFlag')} \
            == extra, name
        assert r.tail_exempt == rows.LIOSAM_TAIL == 2
    # 기준 행의 평면 사전은 꺼져 있어야 6DoF 기준이 된다
    assert base['z_tollerance'] >= 1000.0 and base['rotation_tollerance'] >= 1000.0
    assert base['imuRPYWeight'] == pytest.approx(0.01)


def test_rtabmap_rows_imu_subscription_matches_play_topics():
    """IMU 를 빼는 행은 IMU 를 틀지도 기다리지도 않고, IMU 를 쓰는 행은 그 IMU 를 튼다 (리서치 R11)."""
    mod = _load_launch('rtabmap.launch.py')
    assert mod.resolve_imu('none', 'amr_01') == '' and mod.resolve_imu('', 'amr_01') == ''
    assert mod.resolve_imu('default', 'amr_01') == '/amr_01/imu/data_madgwick'
    assert mod.resolve_imu('/x/imu/data_madgwick_re', 'amr_01') == '/x/imu/data_madgwick_re'
    for name, r in rows.ROWS.items():
        if not name.startswith('rtabmap'):
            continue
        a = _launch_args(r)
        imu = mod.resolve_imu(a.get('imu_topic', 'default'), 'amr_01')
        setup = mod.node_setup(a['dim'], 'amr_01', '/c', '/s', imu, True, 'slam_map', '/db', 'p',
                               'i')
        assert setup[0][1]['wait_imu_to_init'] is bool(imu), name
        imu_played = [t for t in r.play_topics if 'imu' in t]
        if name == 'rtabmap3d_imulive':
            continue                                  # madgwick 입력(imu/data)을 틀고 출력을 구독
        assert imu_played == ([imu] if imu else []), name
        assert r.tail_exempt == 0, name
    assert 'imu' not in ' '.join(rows.row('rtabmap3d_noimu').play_topics)
    assert 'imu' not in ' '.join(rows.row('rtabmap2d_noimu').play_topics)


def test_missing_play_topics_against_bag_metadata():
    """파생 bag 토픽을 원 bag 으로 돌리면 재생 전에 잡는다 (replay_row.sh 의 사전 검사)."""
    meta = """rosbag2_bagfile_information:
  version: 5
  topics_with_message_count:
    - topic_metadata:
        name: /clock
        type: rosgraph_msgs/msg/Clock
      message_count: 3
    - topic_metadata:
        name: /tf_static
        type: tf2_msgs/msg/TFMessage
      message_count: 1
    - topic_metadata:
        name: /amr_01/lidar3d/points
        type: sensor_msgs/msg/PointCloud2
      message_count: 3
    - topic_metadata:
        name: /amr_01/imu/data_madgwick
        type: sensor_msgs/msg/Imu
      message_count: 3
"""
    topics = rows.bag_topics(meta)
    assert topics == ['/clock', '/tf_static', '/amr_01/lidar3d/points',
                      '/amr_01/imu/data_madgwick']
    assert rows.missing_play_topics(rows.row('liosam_loop'), topics) == []
    assert rows.missing_play_topics(rows.row('liosam_planar'), topics) == [rows.IMU_MADG_RE]
    assert rows.missing_play_topics(rows.row('rtabmap3d_noimu'), topics) == ['/tf']


# ---- b2-rec: 재생 시작 전 구독 확인 (replay.md §9-4) ------------------------------------------------
# Humble `ros2 topic info -v` 실제 출력 (b2-rec 탐색: 멈춘 재생기 + 구독 없음) 에 구독자 블록을 더한 것
_INFO = """Type: sensor_msgs/msg/PointCloud2

Publisher count: 1

Node name: rosbag2_player
Node namespace: /
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: PUBLISHER
GID: 01.0f.70.b7.17.00.e8.48.00.00.00.00.00.00.13.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 2

Node name: icp_odometry
Node namespace: /amr_01
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: SUBSCRIPTION
GID: 01.0f.70.b7.18.00.e8.48.00.00.00.00.00.00.14.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE

Node name: slam_probe
Node namespace: /
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: SUBSCRIPTION
GID: 01.0f.70.b7.19.00.e8.48.00.00.00.00.00.00.15.04.00.00.00.00.00.00.00.00
"""


def test_parse_topic_info_humble_format():
    p = rows.parse_topic_info(_INFO)
    assert p == {'publishers': ['rosbag2_player'], 'subscribers': ['icp_odometry', 'slam_probe']}
    assert rows.parse_topic_info('') == {'publishers': [], 'subscribers': []}


def test_unmet_lists_missing_subscribers_and_player():
    reqs = (('/amr_01/lidar3d/points', 'icp_odometry'), ('/amr_01/lidar3d/points', 'rtabmap'))
    miss = rows.unmet(reqs, {'/amr_01/lidar3d/points': _INFO},
                      player_topics=['/amr_01/lidar3d/points'])
    assert miss == ['/amr_01/lidar3d/points ← rtabmap']
    no_player = _INFO.replace('Endpoint type: PUBLISHER', 'Endpoint type: SUBSCRIPTION')
    miss2 = rows.unmet(reqs[:1], {'/amr_01/lidar3d/points': no_player},
                       player_topics=['/amr_01/lidar3d/points'])
    assert miss2 == ['/amr_01/lidar3d/points → rosbag2_player']
    assert rows.unmet(reqs[:1], {}, ()) == ['/amr_01/lidar3d/points ← icp_odometry']


def test_every_row_has_ready_requirements_on_played_topics():
    """모든 행: 입력 토픽에 측정 노드·SLAM 노드, IMU 를 트는 행은 IMU 구독도, 처리 출력에는 C++ 기록기."""
    for name, r in rows.ROWS.items():
        req = rows.ready_requirements(r)
        topics = {t for t, _ in req}
        assert (r.input_topic, rows.PROBE_NODE) in req, name
        assert len([1 for t, n in req if t == r.input_topic and n != rows.PROBE_NODE]) >= 1, name
        for t in topics - {t for t, _, _ in r.processed}:
            assert t in r.play_topics or t == rows.IMU_MADGWICK, (name, t)
        imu = [t for t in r.play_topics if '/imu/' in t]
        if imu:
            assert any(t == imu[0] for t, _ in req), name
        for t, _, _ in r.processed:
            assert (t, rows.RECORDER_NODE) in req, name
    lio = rows.ready_requirements(rows.row('liosam_planar'))
    assert (rows.IMU_MADG_RE, 'lio_sam_imu_preintegration') in lio
    assert (rows.CLOUD, 'lio_sam_imageProjection') in lio
    noimu = rows.ready_requirements(rows.row('rtabmap3d_noimu'))
    assert not any('/imu/' in t for t, _ in noimu)


def test_record_qos_overrides_cover_all_volatile_record_topics():
    """기록기 구독 깊이: /tf_static(transient_local) 을 뺀 모든 기록 토픽, reliable·volatile, 고빈도는 ≥ 1000."""
    q = yaml.safe_load(rows.record_qos_yaml())
    assert set(q) == set(rows.RECORD_TOPICS) - {'/tf_static'}
    for t, p in q.items():
        assert p == {'history': 'keep_last', 'depth': rows.RECORD_QOS_DEPTH[t],
                     'reliability': 'reliable', 'durability': 'volatile'}, t
    for t in ('/clock', '/tf', '/amr_01/imu/data_raw', '/amr_01/imu/data', '/amr_01/joint_states'):
        assert q[t]['depth'] >= 1000
