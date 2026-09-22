"""
시나리오 03: SLAM 맵 생성 (명세 4.3 "해상도 0.05 m 이하, 구조물 일치").

스택(system 프로필): system.launch.py use_simulation + use_localization(mode:=slam) —
slam_toolbox 가 scan_filtered 로 /map 을 만든다 (components.md §3.2 매핑 모드).
주행: GT 되먹임 웨이포인트 추종(actions.follow_waypoints)으로 랙 통로 세 개를 한 바퀴 돈다.
판정: /map 해상도 ≤ 0.05 m, 월드 SDF 정적 구조물(worldmap.footprints — LiDAR 평면 높이) 대비
보이는 가장자리 재현율 ≥ 0.9, 자유 공간 오점유율 ≤ 5 %. slam_toolbox/save_map 으로 저장까지 확인.
필요 노드(amr_localization localization.launch.py, slam_toolbox)가 없으면 사유와 함께 건너뛴다.
"""

import json

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


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements(use_navigation=False, use_perception=False)
                + [req.executable('slam_toolbox', 'async_slam_toolbox_node', 'SLAM 매핑')],
                'SLAM mapping')
    stack = Stack(CTX, CTX.select_backend(), CTX.select_profile())
    stack.system(use_localization=True, use_navigation=False, use_perception=False,
                 extra_args={'mode': 'slam'})
    return stack.launch_description(), {'stack': stack}


class TestSlamMap(cases.ProbeCase):
    """매핑 주행 → /map 해상도·구조물 일치."""

    CTX = CTX

    def test_10_map_quality(self, stack) -> None:
        gt = self.probe.subscribe('ground_truth/odom', Odometry)
        grid_rec = self.probe.subscribe('/map', OccupancyGrid, 'latched', keep_messages=2)
        self.require_topic(gt, 5, 300.0)
        reached = actions.follow_waypoints(self.probe, gt, stack.drive_topic, ROUTE, v=0.6,
                                           timeout_per_wp=self.timeout(180.0))
        self.measure('route_reached', reached)
        self.require_topic(grid_rec, 1, 60.0, '/map (slam_toolbox)')
        grid = grid_rec.last()
        info = grid.info
        data = np.asarray(grid.data, dtype=int).reshape(info.height, info.width)
        self.ctx.record.write_text('map_info.json', json.dumps({
            'resolution': info.resolution, 'width': info.width, 'height': info.height,
            'origin': [info.origin.position.x, info.origin.position.y]}))
        plane_z = (config.get(config.robot_params(), 'robot.base_link_height', 0.18)
                   + config.get(config.sensors(), 'lidar.extrinsic.z', 0.20))
        share = req.share_dir('amr_simulation')
        shapes = worldmap.footprints(share / 'worlds' / self.settings.world,
                                     share / 'models', plane_z)
        agree = worldmap.map_agreement(data, info.resolution,
                                       (info.origin.position.x, info.origin.position.y), shapes)
        self.measure('map_check', {'resolution': info.resolution, 'width': info.width,
                                   'height': info.height, 'recall': cases.fmt(agree.recall),
                                   'false_occupied': cases.fmt(agree.false_occupied),
                                   'edge_points': agree.edge_points,
                                   'occupied_cells': agree.occupied_cells,
                                   'shapes': len(shapes)})
        self.check('map resolution', info.resolution, RESOLUTION_MAX,
                   info.resolution <= RESOLUTION_MAX + 1e-9, 'm')
        self.check('structure recall', agree.recall, RECALL_MIN, agree.recall >= RECALL_MIN)
        self.check('false occupied ratio', agree.false_occupied, FALSE_OCC_MAX,
                   agree.false_occupied <= FALSE_OCC_MAX)

    def test_20_save_map(self) -> None:
        """slam_toolbox/save_map 로 지도 저장 (maps/ 대신 시나리오 로그 디렉토리)."""
        from slam_toolbox.srv import SaveMap
        req_msg = SaveMap.Request()
        req_msg.name.data = str(self.ctx.path('map'))
        res = self.probe.call(SaveMap, 'slam_toolbox/save_map', req_msg, self.timeout(60.0))
        self.assertIsNotNone(res, 'slam_toolbox/save_map 응답 없음')
        saved = self.ctx.path('map.pgm').is_file() and self.ctx.path('map.yaml').is_file()
        self.check('map saved', saved, True, saved)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
