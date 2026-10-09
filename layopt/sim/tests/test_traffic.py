"""TrafficManager 연결: FastZoneMap 이 원본 ZoneMap 과 같은 답을 내고, E1TrafficManager 가 원본과 같은 명령을 낸다."""

import random

import numpy as np

from conftest import needs_core
from layopt.sim.sim import E1Sim, SimConfig
from layopt.sim.traffic import E1TrafficManager, FastZoneMap, load_config, TRAFFIC_YAML
from amr_fleet.traffic_manager import TrafficManager
from amr_fleet.traffic_zones import load_layout, ZoneMap


def _zones(layout):
    zones, _ = load_layout(str(layout.zones_yaml), True)
    return zones


def test_fast_zone_map_equals_reference(layout):
    zones = _zones(layout)
    ref, fast = ZoneMap(zones), FastZoneMap(zones)
    assert fast._fast
    rng = random.Random(1)
    pts = [(rng.uniform(-31, 31), rng.uniform(-21, 21)) for _ in range(20000)]
    for z in zones:                                   # 경계 · 모서리 · 그 바로 안팎
        xs = sorted(set(float(v) for v in z.polygon[:, 0]))
        ys = sorted(set(float(v) for v in z.polygon[:, 1]))
        for x in xs + [xs[0] - 1e-9, xs[1] + 1e-9, xs[0] + 1e-9, xs[1] - 1e-9, sum(xs) / 2]:
            for y in ys + [ys[0] - 1e-9, ys[1] + 1e-9, ys[0] + 1e-9, ys[1] - 1e-9, sum(ys) / 2]:
                pts.append((x, y))
    a = np.array(pts)
    assert fast.zones_at(a) == ref.zones_at(a)


def test_fast_zone_map_overlap_first_defined_wins():
    from amr_fleet.traffic_zones import zones_from_config
    cfg = [{'id': 'a', 'kind': 'intersection', 'polygon': [[0, 0], [2, 0], [2, 2], [0, 2]]},
           {'id': 'b', 'kind': 'intersection', 'polygon': [[1, 1], [3, 1], [3, 3], [1, 3]]}]
    zs = zones_from_config(cfg, True)
    pts = np.array([[1.5, 1.5], [2.5, 2.5], [0.5, 0.5], [2.0, 2.0], [1.0, 1.0]])
    assert FastZoneMap(zs).zones_at(pts) == ZoneMap(zs).zones_at(pts)


def test_fast_zone_map_falls_back_for_non_rectangles():
    from amr_fleet.traffic_zones import zones_from_config
    cfg = [{'id': 't', 'kind': 'intersection', 'polygon': [[0, 0], [4, 0], [0, 3]]}]
    zs = zones_from_config(cfg, True)
    fm = FastZoneMap(zs)
    assert not fm._fast
    pts = np.array([[0.5, 0.5], [3.0, 2.0]])
    assert fm.zones_at(pts) == ZoneMap(zs).zones_at(pts)


def test_load_config_reads_traffic_yaml(layout):
    cfg = load_config(layout.zones_yaml)
    assert cfg.zones_file == str(layout.zones_yaml)
    assert cfg.zones.approach_distance_m == 2.0 and cfg.zones.corridor_approach_m == 2.5
    assert cfg.deadlock.wait_max_s == 120.0 and cfg.resolution.yield_timeout_s == 30.0
    assert cfg.prediction.crossing_act_s == 4.0 and cfg.mode == 'active'
    assert TRAFFIC_YAML.is_file()


class _TwinSim(E1Sim):
    """E1TrafficManager 와 원본 TrafficManager 에 같은 관측을 넣어 명령 · 사건을 비교한다."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        ref = TrafficManager(self.world.tm_cfg)
        ref.set_map(self.navmap.occupied, self.navmap.spec)
        self.ref = ref
        self.ticks = 0
        self.mismatch = []

    def _traffic(self, now):
        from amr_fleet.traffic_manager import RobotObservation
        obs = {r.id: RobotObservation(r.id, r.x, r.y, r.th, r.v, r.plan_xy(), r.status(),
                                      r.task.task_id if r.task is not None else '', r.priority(),
                                      None, None) for r in self.robots}
        a = self.tm.update(now, obs)
        b = self.ref.update(now, obs)
        self.ticks += 1
        for rid in a.commands:
            ca, cb = a.commands[rid], b.commands[rid]
            if (ca.hold, ca.reason, ca.yield_pose, ca.keepout is None) != \
                    (cb.hold, cb.reason, cb.yield_pose, cb.keepout is None):
                self.mismatch.append((now, rid))
        if [e.name for e in a.events] != [e.name for e in b.events]:
            self.mismatch.append((now, 'events'))
        for rid, cmd in a.commands.items():
            self.by_id[rid].apply_command(now, cmd)


@needs_core
def test_e1_traffic_manager_matches_reference(world):
    sim = _TwinSim(world, SimConfig(workload='p5', horizon_s=420, soc=100.0), 0)
    sim.run()
    assert sim.ticks > 800
    assert sim.mismatch == []
    assert isinstance(sim.tm, E1TrafficManager)
