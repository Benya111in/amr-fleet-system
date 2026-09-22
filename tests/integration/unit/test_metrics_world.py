"""metrics · worldmap · procmon · gz · evaluation (순수 부분)."""

import math
from types import SimpleNamespace as NS

from amr_itest import evaluation, gz, metrics, procmon, worldmap
import numpy as np
import pytest


def _odom(t, x, y, yaw, v=0.0, w=0.0):
    sec = int(t)
    return NS(header=NS(stamp=NS(sec=sec, nanosec=int(round((t - sec) * 1e9)))),
              pose=NS(pose=NS(position=NS(x=x, y=y, z=0.0),
                              orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2),
                                             w=math.cos(yaw / 2)))),
              twist=NS(twist=NS(linear=NS(x=v, y=0.0, z=0.0), angular=NS(z=w))))


def test_classify_and_sample():
    assert metrics.classify_motion(0.0, 0.0) == metrics.STOP
    assert metrics.classify_motion(0.5, 0.01) == metrics.STRAIGHT
    assert metrics.classify_motion(0.0, 0.5) == metrics.TURN
    s = metrics.sample_from_odom(_odom(3.25, 1.0, 2.0, 0.5, 0.3, 0.1))
    assert (s.t, s.x, s.y, s.v, s.w) == pytest.approx((3.25, 1.0, 2.0, 0.3, 0.1))
    assert s.yaw == pytest.approx(0.5)


def test_interpolate_and_pairing():
    gt = [metrics.Sample(t * 0.02, t * 0.01, 0.0, 0.0, 0.5, 0.0) for t in range(100)]
    g = metrics.interpolate(gt, 0.03)
    assert g.x == pytest.approx(0.015)
    assert metrics.interpolate(gt, 0.04) is gt[2]
    assert metrics.interpolate(gt, -1.0) is None and metrics.interpolate([], 0.0) is None
    gap = [metrics.Sample(0.0, 0, 0, 3.0), metrics.Sample(1.0, 1, 0, -3.0)]
    assert metrics.interpolate(gap, 0.5) is None
    wrapped = metrics.interpolate(gap, 0.5, max_gap=2.0)
    assert abs(wrapped.yaw) == pytest.approx(math.pi, abs=0.15)        # 최단 각으로 보간
    est = [metrics.Sample(t * 0.02 + 0.01, t * 0.01 + 0.005 + 0.02, 0.0, 0.0)
           for t in range(99)] + [metrics.Sample(10.0, 0, 0, 0)]
    rows = metrics.pair_pose_errors(gt, est)
    assert len(rows) == 99
    assert all(r.error == pytest.approx(0.02) and r.segment == metrics.STRAIGHT for r in rows)
    assert len(rows[0].as_list()) == len(metrics.POSE_COLUMNS)
    summ = metrics.pose_summary(rows)
    assert summ[metrics.STRAIGHT].rmse == pytest.approx(0.02)
    assert summ[metrics.STOP].count == 0 and summ['전체'].count == 99
    verdict = {v[0]: v for v in metrics.pose_verdicts(summ, min_samples=50)}
    assert verdict[metrics.STRAIGHT][3] and not verdict[metrics.STOP][3]
    assert not dict((v[0], v[3]) for v in metrics.pose_verdicts(summ))[metrics.STRAIGHT]
    d = summ[metrics.STRAIGHT].as_dict(scale=100.0, digits=2)
    assert d['rmse'] == pytest.approx(2.0) and math.isnan(summ[metrics.STOP].as_dict()['rmse'])


def test_speeds_at_contact_classification():
    # 0~1 s 정지, 1~2 s 0.4 m/s (08 접촉 분류: 움직이며 부딪침 vs 멈춘 로봇에 닿음)
    track = [metrics.Sample(k * 0.02, 0.0, 0.0, 0.0, 0.0 if k * 0.02 < 1.0 else 0.4, 0.0)
             for k in range(101)]
    v = metrics.speeds_at(track, [0.5, 1.5, 5.0])
    assert v[0] == pytest.approx(0.0) and v[1] == pytest.approx(0.4) and math.isnan(v[2])
    assert v[0] <= metrics.CONTACT_MOVING_V < v[1]
    assert metrics.speeds_at([], [0.0])[0] != metrics.speeds_at([], [0.0])[0]     # NaN


def test_lane_coords_contact_attribution():
    # +x 로 1 m/s 걷는 작업자, 로봇은 2 m 앞 · 왼쪽 0.5 m (08 접촉 원인 귀속)
    along, lat = metrics.lane_coords(0.0, 0.0, 1.0, 0.0, 2.0, 0.5)
    assert along == pytest.approx(2.0) and lat == pytest.approx(0.5)
    # +y 로 걸으면 왼쪽은 −x 쪽: 같은 로봇 위치가 오른쪽(−)·앞(+0.5) 으로 바뀐다
    along, lat = metrics.lane_coords(0.0, 0.0, 0.0, 1.0, 2.0, 0.5)
    assert along == pytest.approx(0.5) and lat == pytest.approx(-2.0)
    # 대각선 진행이어도 |가로| 는 진행축까지의 수직 거리
    _, lat = metrics.lane_coords(0.0, 0.0, 1.0, 1.0, 1.0, 0.0)
    assert lat == pytest.approx(-math.sqrt(0.5))
    # 멈춰 있는 장애물은 진행축이 없다 → NaN (차선 좌표를 쓰지 않는다)
    assert all(math.isnan(c) for c in metrics.lane_coords(0.0, 0.0, 0.0, 0.0, 1.0, 1.0))


def test_first_motion_and_latency():
    speeds = ((0.0, 0.0), (0.1, 0.01), (0.2, 0.06), (0.3, 0.2))
    track = [metrics.Sample(t, 0, 0, 0, v, 0.0) for t, v in speeds]
    assert metrics.first_motion(track, 0.05).t == 0.2
    assert metrics.first_motion(track, 0.25).t == 0.3
    assert metrics.first_motion(track[:2], 0.0) is None
    s = metrics.latency_summary([100.0, 120.0, float('nan'), 80.0])
    assert s['count'] == 3 and s['mean'] == 100.0 and s['max'] == 120.0 and s['min'] == 80.0
    assert metrics.latency_summary([])['count'] == 0
    assert metrics.wrap(4 * math.pi + 0.1) == pytest.approx(0.1)


WORLD = """<?xml version="1.0"?>
<sdf version="1.8"><world name="w">
  <model name="walls"><static>true</static><pose>0 0 0 0 0 0</pose>
    <link name="l"><collision name="c"><pose>0 5 1 0 0 0</pose>
      <geometry><box><size>10 0.2 2</size></box></geometry></collision>
      <collision name="low"><pose>0 -5 0.05 0 0 0</pose>
      <geometry><box><size>10 0.2 0.1</size></box></geometry></collision></link></model>
  <model name="robot_like"><pose>1 1 0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>1 1 1</size></box></geometry>
    </collision></link></model>
  <include><uri>model://pillar</uri><name>pillar_1</name><pose>2 0 0 0 0 1.5708</pose></include>
  <include><uri>model://missing</uri><name>m</name></include>
  <include><uri>file://x</uri></include>
  <actor name="worker"><script><loop>true</loop><delay_start>1.0</delay_start>
    <trajectory id="0" type="walk">
      <waypoint><time>0</time><pose>0 0 0 0 0 0</pose></waypoint>
      <waypoint><time>2</time><pose>2 0 0 0 0 0</pose></waypoint>
    </trajectory></script></actor>
  <actor name="idle"/>
</world></sdf>
"""
PILLAR = """<?xml version="1.0"?>
<sdf version="1.8"><model name="pillar"><static>true</static>
  <link name="l"><pose>0 0 1 0 0 0</pose><collision name="c">
  <geometry><cylinder><radius>0.25</radius><length>3</length></cylinder></geometry>
  </collision></link></model></sdf>
"""


@pytest.fixture
def world(tmp_path):
    (tmp_path / 'models' / 'pillar').mkdir(parents=True)
    (tmp_path / 'models' / 'pillar' / 'model.sdf').write_text(PILLAR)
    (tmp_path / 'w.sdf').write_text(WORLD)
    return tmp_path


def test_footprints_and_agreement(world):
    shapes = worldmap.footprints(world / 'w.sdf', world / 'models', 0.20, 'collision',
                                 static_only=True)
    kinds = sorted(type(s).__name__ for s in shapes)
    assert kinds == ['Circle', 'Rect']                     # 낮은 벽·비정적 모델 제외
    rect = next(s for s in shapes if isinstance(s, worldmap.Rect))
    circle = next(s for s in shapes if isinstance(s, worldmap.Circle))
    assert (rect.cx, rect.cy, rect.sx) == pytest.approx((0.0, 5.0, 10.0))
    assert (circle.cx, circle.cy, circle.r) == pytest.approx((2.0, 0.0, 0.25))
    res, origin, w, h = 0.05, (-6.0, -6.0), 240, 240
    gt = worldmap.rasterize(shapes, w, h, res, origin)
    grid = np.where(gt, 100, 0)
    perfect = worldmap.map_agreement(grid, res, origin, shapes)
    assert perfect.recall == pytest.approx(1.0) and perfect.false_occupied == pytest.approx(0.0)
    noisy = grid.copy()
    noisy[10:20, 10:20] = 100                                # 없는 구조물
    noisy[gt & (np.arange(w)[None, :] > 120)] = 0            # 벽 오른쪽 절반 누락
    bad = worldmap.map_agreement(noisy, res, origin, shapes)
    assert bad.recall < 0.8 and bad.false_occupied > 0.05
    assert bad.hotspots[0][:2] == (-5.5, -5.5) and bad.hotspots[0][2] == 100   # 없는 구조물 자리
    assert perfect.hotspots == ()
    pgm, yml = worldmap.write_map(noisy, res, origin, world / 'saved')
    raw = pgm.read_bytes()
    assert raw.startswith(f'P5\n{w} {h}\n255\n'.encode()) and len(raw) > w * h
    body = np.frombuffer(raw[-w * h:], dtype=np.uint8).reshape(h, w)[::-1]
    assert body[15, 15] == 0 and body[0, 0] == 254 and 'trinary' in yml.read_text()
    unknown = worldmap.map_agreement(np.full((h, w), -1), res, origin, shapes)
    assert math.isnan(unknown.recall) and math.isnan(unknown.false_occupied)
    pts = worldmap.sample_free(shapes, (-5, -5, 5, 5), 0.5, 20, np.random.default_rng(1))
    assert len(pts) == 20
    assert not any(bool(s.contains(pts[:, 0], pts[:, 1], 0.49).any()) for s in shapes)


def test_polyline_actor_include(world):
    line = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]])
    assert worldmap.polyline_distance(line, 1.0, 0.5) == pytest.approx(0.5)
    assert worldmap.polyline_distance(line, 3.0, 3.0) == pytest.approx(math.sqrt(2))
    assert worldmap.polyline_distance(line[:1], 3.0, 4.0) == pytest.approx(5.0)
    assert math.isinf(worldmap.polyline_distance(np.zeros((0, 2)), 0, 0))
    actors = worldmap.actor_tracks(world / 'w.sdf')
    assert [a.name for a in actors] == ['worker']
    a = actors[0]
    assert a.position(0.5) == pytest.approx((0.0, 0.0))          # delay_start 전
    assert a.position(2.0) == pytest.approx((1.0, 0.0))
    assert a.position(4.0) == pytest.approx((1.0, 0.0))          # loop
    once = worldmap.ActorTrack('o', (0.0, 2.0), ((0.0, 0.0), (2.0, 0.0)), loop=False)
    assert once.position(10.0) == pytest.approx((2.0, 0.0))
    late = worldmap.ActorTrack('l', (1.0, 2.0), ((0.0, 0.0), (2.0, 0.0)), loop=False)
    assert late.position(0.5) == pytest.approx((0.0, 0.0))
    assert all(math.isnan(v) for v in worldmap.ActorTrack('e', (), ()).position(1.0))
    assert worldmap.include_pose(world / 'w.sdf', 'pillar_1')[:2] == pytest.approx((2.0, 0.0))
    assert worldmap.include_pose(world / 'w.sdf', 'walls') == (0.0, 0.0, 0.0, 0.0)
    assert worldmap.include_pose(world / 'w.sdf', 'nope') is None
    assert worldmap.parse_pose(None) == (0.0, 0.0, 0.0, 0.0)
    assert worldmap.compose((1, 0, 0, math.pi / 2), (1, 0, 0, 0))[:2] == pytest.approx((1, 1))


@pytest.fixture
def fake_proc(tmp_path):
    p = tmp_path / 'proc'
    p.mkdir()
    (p / 'stat').write_text('cpu  100 0 100 800 0 0 0 0 0 0\ncpu0 1 1 1 1\n')
    for pid, args, rss, jiffies in ((10, ['/opt/ros/lib/ekf_node', '--ros-args', '-r',
                                          '__node:=ekf_filter_node_odom'], 20480, (5, 5)),
                                    (11, ['python3', 'x.py'], 1024, (1, 1)),
                                    (12, ['/usr/bin/rsp', '--ros-args'], 2048, (0, 0))):
        d = p / str(pid)
        d.mkdir()
        (d / 'cmdline').write_bytes(b'\x00'.join(a.encode() for a in args) + b'\x00')
        (d / 'status').write_text(f'Name:\tx\nVmRSS:\t{rss} kB\n')
        (d / 'stat').write_text(f'{pid} (a b) S ' + ' '.join(['0'] * 10)
                                + f' {jiffies[0]} {jiffies[1]} 0 0\n')
    (p / 'self').mkdir()
    return p


def test_procmon(fake_proc):
    procs = procmon.ros_processes(fake_proc)
    assert procs == {10: 'ekf_filter_node_odom', 12: 'rsp'}
    d = fake_proc / '13'                          # Gazebo 서버 (ruby /usr/bin/ign gazebo -s …)
    d.mkdir()
    (d / 'cmdline').write_bytes(b'ruby\x00/usr/bin/ign\x00gazebo\x00-s\x00w.sdf\x00')
    (d / 'stat').write_text('13 (ruby) S ' + ' '.join(['0'] * 10) + ' 0 0 0 0\n')
    # 시스템 = ROS 노드 + 시뮬레이터 (명세 4.10 CPU: 시뮬레이터를 빼면 기준이 헐거워진다)
    assert procmon.system_processes(fake_proc) == {10: 'ekf_filter_node_odom', 12: 'rsp',
                                                   13: 'gazebo_server'}
    assert not procmon.is_simulator(['ign', 'service', '-s', '/world/w/create'])
    assert procmon.is_simulator(['/opt/ros/lib/ign-gazebo-server'])
    assert procmon.cpu_count(fake_proc) == 1
    assert procmon.rss_mb(10, fake_proc) == pytest.approx(20.0)
    assert procmon.rss_mb(99, fake_proc) is None
    (fake_proc / '11' / 'status').write_text('Name:\tx\n')
    assert procmon.rss_mb(11, fake_proc) is None
    assert procmon.node_name([]) == ''
    cpu = procmon.CpuSampler(fake_proc)
    own = procmon.ProcessCpuSampler([10, 99], fake_proc)
    assert cpu.sample() == 0.0 and own.sample() == 0.0           # 변화 없음
    (fake_proc / 'stat').write_text('cpu  200 0 200 1600 0 0 0 0 0 0\n')
    (fake_proc / '10' / 'stat').write_text('10 (a b) S ' + ' '.join(['0'] * 10)
                                           + ' 55 55 0 0\n')
    assert cpu.sample() == pytest.approx(20.0)
    assert own.sample() == pytest.approx(10.0)
    assert own.cores_used() == pytest.approx(0.1)          # 코어 1개 호스트의 10 %
    own.update_pids([10, 13])                              # 새로 뜬 프로세스는 다음 표본부터
    (fake_proc / 'stat').write_text('cpu  300 0 300 2400 0 0 0 0 0 0\n')
    (fake_proc / '13' / 'stat').write_text('13 (ruby) S ' + ' '.join(['0'] * 10) + ' 40 0 0 0\n')
    assert own.sample() == pytest.approx(0.0 + 100.0 * 40 / 1000)
    assert procmon.process_jiffies(99, fake_proc) is None
    assert procmon.slope_per_hour([0, 1800, 3600], [10.0, 12.5, 15.0]) == pytest.approx(5.0)
    assert math.isnan(procmon.slope_per_hour([0, 1], [1, 2]))


def test_leak_verdict_noise_floor_and_insufficient():
    # 14 스모크 실측: 창 4 분의 1.5 MB 왕복 요동 → 18 MB/h 로 외삽되지만 누수 아님
    t = [600, 660, 720, 780, 840]
    state, slope, growth = procmon.leak_verdict(t, [142.6, 144.1, 142.6, 144.1, 144.1], 5.0)
    assert state == 'ok' and slope > 5.0 and growth < procmon.RSS_NOISE_MB
    # 같은 창에서 꾸준히 7 MB 증가 (planner_server 115 → 122 MB) → 누수
    state, slope, growth = procmon.leak_verdict(t, [115.0, 115.0, 116.2, 120.6, 122.2], 5.0)
    assert state == 'leak' and growth > procmon.RSS_NOISE_MB
    # 4 h 창에서 5 MB/h 초과 꾸준한 증가는 요동 바닥과 무관하게 누수
    hours = [600 + 900 * k for k in range(16)]
    state, slope, _ = procmon.leak_verdict(hours, [100 + 6.0 * h / 3600 for h in hours], 5.0)
    assert state == 'leak' and slope == pytest.approx(6.0)
    # 평탄하면 ok, 표본 부족·NaN 은 판정 불가 (통과로 세지 않는다)
    assert procmon.leak_verdict(hours, [100.0] * 16, 5.0)[0] == 'ok'
    assert procmon.leak_verdict([600, 660], [1.0, 2.0], 5.0)[0] == 'insufficient'
    assert procmon.leak_verdict(t, [1.0, float('nan'), float('nan'), 2.0, 3.0], 5.0)[0] \
        == 'insufficient'


def test_gz_requests(monkeypatch):
    text = gz.pose_request('amr_01', 1.0, 2.0, 0.02, math.pi)
    assert 'name: "amr_01"' in text and 'z: 1.000000' in text
    sdf = gz.box_sdf('blk', 1.0, 2.0, 1.0, 3.0, 1.0)
    assert '"' not in sdf and '<size>1.0 3.0 1.0</size>' in sdf
    calls = []

    def fake_run(cmd, **kw):
        calls.append(cmd)
        return NS(returncode=0, stdout='data: true\n', stderr='')
    monkeypatch.setattr(gz.subprocess, 'run', fake_run)
    assert gz.set_pose('warehouse', 'amr_01', 0, 0, 0)[0]
    assert gz.spawn_box('warehouse', 'blk', 0, 0)[0]
    assert gz.remove('warehouse', 'blk')[0]
    assert [c[3] for c in calls] == ['/world/warehouse/set_pose', '/world/warehouse/create',
                                     '/world/warehouse/remove']

    def missing(cmd, **kw):
        raise FileNotFoundError('ign')
    monkeypatch.setattr(gz.subprocess, 'run', missing)
    ok, out = gz.set_pose('w', 'm', 0, 0, 0)
    assert not ok and 'ign' in out


def test_evaluation_analyze(tmp_path):
    """amr_evaluation 이 설치된 워크스페이스에서 CSV → analyze 판정 행."""
    pytest.importorskip('amr_evaluation')
    rows = [[i * 0.02, 0.0, 0.0, 0.01, 0.0, 0.01, 0.0, seg]
            for i, seg in enumerate(['정지'] * 120 + ['직선'] * 120 + ['회전'] * 120)]
    with open(tmp_path / 'pose_error.csv', 'w') as fh:
        fh.write(','.join(metrics.POSE_COLUMNS) + '\n')
        fh.writelines(','.join(str(v) for v in r) + '\n' for r in rows)
    # response_time_logger 의 실제 열 (io.RESPONSE_COLUMNS): rtf · latency_wall_ms 가 있어야 analyze 가
    # 벽시계 환산 지연으로 판정한다 (없으면 경고와 함께 전체 판정을 보류 — 예전 이 시험은 그 열을 빼서
    # 설치본 amr_evaluation 에 따라 결과가 달랐다)
    cols = metrics.RESPONSE_COLUMNS + ['cmd_source', 'rtf', 'latency_wall_ms']
    with open(tmp_path / 'response_time.csv', 'w') as fh:
        fh.write(','.join(cols) + '\n')
        fh.writelines(f'{i},{i + 0.1},100.0,t{i},event,1.0,100.0\n' for i in range(50))
    result = evaluation.analyze(tmp_path)
    pose = evaluation.gated_rows(result, '위치 추정 오차')
    assert len(pose) == 3 and all(r['passed'] for r in pose)
    resp = evaluation.gated_rows(result, '응답 시간')
    assert resp[0]['count'] == 50 and resp[0]['value'] == pytest.approx(100.0)
    assert (tmp_path / 'report.md').is_file()
    # 런 전체 판정(passed)은 이 합성 런에 없는 지표 계열(CTE·CPU)을 데이터 부족으로 본다 — 시나리오가
    # result['passed'] 가 아니라 판정 행으로 판정하는 이유. 부족 사유는 그 계열뿐이어야 한다
    assert not result['warnings'], result['warnings']
    assert result['insufficient']
    assert not any('위치' in w or '응답' in w for w in result['insufficient'])


def test_evaluation_helpers(monkeypatch):
    rows = {'rows': [{'metric': '위치 추정 오차', 'passed': True},
                     {'metric': '위치 추정 오차 (헤딩)', 'passed': None},
                     {'metric': '응답 시간', 'passed': False}]}
    assert len(evaluation.gated_rows(rows, '위치 추정 오차')) == 1
    assert evaluation.gated_rows(rows, '응답')[0]['passed'] is False
    assert evaluation._num(float('nan')) is None and evaluation._num(1.5) == 1.5
    monkeypatch.setattr(evaluation.req, 'has_python_module', lambda name: False)
    assert not evaluation.available()
    monkeypatch.setattr(evaluation.req, 'has_python_module', lambda name: True)
    monkeypatch.setattr(evaluation.req, 'has_executable', lambda p, e: e == 'cte_logger')
    assert evaluation.available() and evaluation.available('cte_logger')
    assert not evaluation.available('pose_error_logger')
