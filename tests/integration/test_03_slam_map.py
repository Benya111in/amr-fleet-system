"""
시나리오 03: SLAM 맵 생성 (명세 4.3 "해상도 0.05 m 이하, 구조물 일치").

스택(system 프로필): system.launch.py with_localization(localization_mode:=slam), 스폰 = 월드 원점 yaw 0
(명시) — slam_toolbox 의 map 프레임은 시작 자세이므로 map = 월드가 된다. slam_toolbox 가 scan_filtered 로
/map 을 만든다 (components.md §3.2 매핑 모드). navigation·perception 은 끄고 cmd_vel 을 직접 낸다.
주행: GT 되먹임 웨이포인트 추종(actions.follow_waypoints)으로 랙 통로 세 개를 한 바퀴 돈다
(구간마다 대기 상한 = 거리/속도 × 2 + 20 s, sim time — 부하로 RTF 가 떨어져도 구간이 잘리지 않고,
sim time 이 멈추면 wall 10 배에서 포기).
판정: /map 해상도 ≤ 0.05 m, 월드 SDF visual(gpu_lidar 가 재는 면)의 LiDAR 평면 단면(worldmap.footprints) 대비
보이는 가장자리 재현율 ≥ 0.9, 자유 공간 오점유율 ≤ 5 %, map ↔ 월드 SE(2) 잔여 정합 (항등 확인),
map_saver_server 로 저장까지 확인 (판정에 쓴 /map 은 하네스가 map_received.pgm/yaml 로 따로 남긴다 —
오점유가 몰린 1 m 칸 상위 10 개도 map_check 에 기록).
"""

import json
import math

from amr_itest import actions, cases, catalog, config, worldmap
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import pytest

CTX = Context(catalog.get(3))

RESOLUTION_MAX = 0.05
RECALL_MIN = 0.90
FALSE_OCC_MAX = 0.05
# 랙 열 사이 통로(y = 6, 0, -6)와 랙 끝(x = ±21, 랙 베이는 x ±19 까지)·북측(y = 13, 기둥 (±10, 12)
# 에서 1 m) 을 도는 경로 [m] (gen_warehouse_world.py 좌표계, 원점 = 기본 스폰)
ROUTE = [(0.0, 0.0), (21.0, 0.0), (21.0, 6.0), (-21.0, 6.0), (-21.0, -6.0), (21.0, -6.0),
         (21.0, 13.0), (-21.0, 13.0), (-21.0, 0.0), (0.0, 0.0)]
SPEED = 0.6


def leg_timeouts(route, v: float = SPEED, factor: float = 2.0, margin: float = 20.0):
    """구간마다 대기 상한 [s] = 거리 / v × factor + margin (첫 점은 시작 자세에서 0 m)."""
    out, prev = [], route[0]
    for p in route:
        out.append(math.hypot(p[0] - prev[0], p[1] - prev[1]) / v * factor + margin)
        prev = p
    return out


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements(use_navigation=False, use_perception=False)
                + [req.executable('slam_toolbox', 'async_slam_toolbox_node', 'SLAM 매핑')],
                'SLAM mapping')
    stack = Stack(CTX, *CTX.select())
    stack.system(use_localization=True, use_navigation=False, use_perception=False,
                 extra_args={'localization_mode': 'slam'}, pose=(0.0, 0.0, 0.0))
    return stack.launch_description(), {'stack': stack}


class TestSlamMap(cases.ProbeCase):
    """매핑 주행 → /map 해상도·구조물 일치."""

    CTX = CTX

    def test_10_map_quality(self, stack) -> None:
        gt = self.probe.subscribe('ground_truth/odom', Odometry)
        grid_rec = self.probe.subscribe('/map', OccupancyGrid, 'latched', keep_messages=2)
        self.require_topic(gt, 5, 300.0)
        self.wait_startup_still()
        reached = []
        for wp, limit in zip(ROUTE, leg_timeouts(ROUTE)):
            reached += actions.follow_waypoints(self.probe, gt, stack.drive_topic, [wp], v=SPEED,
                                                timeout_per_wp=self.timeout(limit))
            self.measure('route_reached', reached)
        self.require_topic(grid_rec, 1, 60.0, '/map (slam_toolbox)')
        # 마지막 지도 갱신(map_update_interval 2 s)을 받는다
        self.probe.sleep_ros(3.0, self.timeout(60.0))
        grid = grid_rec.last()
        info = grid.info
        data = np.asarray(grid.data, dtype=int).reshape(info.height, info.width)
        self.ctx.record.write_text('map_info.json', json.dumps({
            'resolution': info.resolution, 'width': info.width, 'height': info.height,
            'origin': [info.origin.position.x, info.origin.position.y]}))
        plane_z = config.scan_plane_height()
        share = req.share_dir('amr_simulation')
        shapes = worldmap.footprints(share / 'worlds' / self.settings.world,
                                     share / 'models', plane_z, 'visual')
        origin = (info.origin.position.x, info.origin.position.y)
        worldmap.write_map(data, info.resolution, origin, self.ctx.path('map_received'))
        agree = worldmap.map_agreement(data, info.resolution, origin, shapes)
        self.measure('map_check', {'resolution': info.resolution, 'width': info.width,
                                   'height': info.height, 'recall': cases.fmt(agree.recall),
                                   'false_occupied': cases.fmt(agree.false_occupied),
                                   'edge_points': agree.edge_points,
                                   'occupied_cells': agree.occupied_cells,
                                   'shapes': len(shapes), 'scan_plane_z': plane_z,
                                   'false_occupied_hotspots_1m': [list(h) for h in
                                                                  agree.hotspots]})
        failed = []
        for name, value, thr, ok in (
                ('map resolution', info.resolution, RESOLUTION_MAX,
                 info.resolution <= RESOLUTION_MAX + 1e-9),
                ('structure recall', agree.recall, RECALL_MIN, agree.recall >= RECALL_MIN),
                ('false occupied ratio', agree.false_occupied, FALSE_OCC_MAX,
                 agree.false_occupied <= FALSE_OCC_MAX),
                ('route waypoints reached', sum(reached), len(ROUTE), all(reached))):
            self.ctx.record.check(name, cases.fmt(value), thr, bool(ok))
            if not ok:
                failed.append(f'{name}={cases.fmt(value)} (기준 {thr})')
        # map 프레임 = 시작 자세 = 월드 원점 → 항등 정합이어야 한다 (지도 품질 + 좌표 정합)
        try:
            self.check_map_registration(grid=grid)
        except AssertionError as exc:
            failed.append(str(exc))
        self.assertFalse(failed, '; '.join(failed))

    def test_20_save_map(self) -> None:
        """
        map_saver_server 로 지도 저장 (maps/ 대신 시나리오 로그 디렉토리).

        slam_toolbox/save_map 이 아니라 우리가 띄운 nav2 map_saver_server 를 쓴다: slam_toolbox 는
        지도를 절대 토픽 /map 에 내면서 저장 때 만드는 내부 map_saver 는 상대 토픽 map
        (= <ns>/map, 발행자 없음) 을 구독해 이름공간 아래에서는 늘 실패한다 (localization.launch.py 주석).
        미지 셀(트라이너리 205, p = 0.196) 이 자유 공간으로 읽히지 않게 free 0.19 를 요청에 싣는다.
        """
        from nav2_msgs.srv import SaveMap
        req_msg = SaveMap.Request()
        req_msg.map_topic = '/map'
        req_msg.map_url = str(self.ctx.path('map'))
        req_msg.image_format = 'pgm'
        req_msg.map_mode = 'trinary'
        req_msg.free_thresh = 0.19
        req_msg.occupied_thresh = 0.65
        res = self.probe.call(SaveMap, 'map_saver_server/save_map', req_msg, self.timeout(60.0))
        self.assertIsNotNone(res, 'map_saver_server/save_map 응답 없음')
        saved = (bool(res.result) and self.ctx.path('map.pgm').is_file()
                 and self.ctx.path('map.yaml').is_file())
        self.check('map saved', saved, True, saved)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
