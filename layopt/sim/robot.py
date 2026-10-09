"""
로봇 1 대: 운동학 · 작업 단계 · 도킹 시간 · 배터리 · 충전 · 교통 명령 · 물리 막힘.

운동학 (nav2_params.yaml · robot_params.yaml limits)
- 팀 A* 경로(NavPath)를 호 길이 s 로 따라간다. v ≤ 1.0 m/s, 가속·감속 1.0 m/s². 곡률 상한 v ≤ ω_max/κ
  (ω_max 1.5 rad/s, 앞 상한까지 미리 감속 — DWA 근사). 출발 방향 차가 45° 넘으면 서서 1.5 rad/s 로 회전
  (rotate_to_heading_min_angle 0.785), 끝에서 목표 방위로 회전 (goal yaw tolerance 0.05 rad).
- 멈출 점(경로 끝 · hold · 몸체 앞)까지 남은 거리 d 에서 v ≤ −a·dt + √((a·dt)² + 2·a·d) (fleetsim.robot 과 같은 이산
  상한). 교통 hold 는 현행 실행기처럼 그 자리에서 최대 감속으로 선다 (TrafficGate 가 주행 goal 을 취소).
작업 (task_executor.xml): to_pickup → perceive(3.3 s) → dock(U(21, 24.5) s) → load(물품 적재 시간) →
  undock(15.8 s) → to_drop → dock → unload → undock → 완료 → return(대기 슬롯) → idle.
  P5 Gazebo 7 회의 phase 사건에서 잰 값이다 (docs/research/layout-opt/sim.md §보정, [측정]).
  도킹 · 적재 · 이탈 동안 차체는 staging 자세에 있다고 본다 (판까지 1.55 m 전진은 넣지 않는다).
충전 (charge.xml, 단 대기열과 충전 뒤 복귀는 과제 지시): idle 이고 잔량 < 20 % → 가장 가까운 빈 충전기로 →
  dock → charging(≥ 80 %) → undock → 대기 슬롯 복귀. 빈 충전기가 없으면 가장 가까운 충전기 대기열 자세에서 기다린다.
몸체 막힘 (물리, 구역 밖 포함): 경로 앞 3 m 안, 옆 간격 < 반폭 합 + 0.3 m 인 몸체(다른 로봇 · actor)는
  - 지나갈 수 있으면(정지 · 마주 옴 · 느린 같은 방향, 비켜야 할 옆 거리 ≤ 0.95 m (DWA max_path_offset_hard),
    그 자리 정적 여유 ≥ 비킬 거리 + 반폭 + 0.1 m, 상대가 교통 hold 로 선 로봇이 아님, 내 정지 목표가 그 앞이 아님)
    0.5 m/s 로 줄여 지나가고,
  - 아니면 그 몸체 앞(세로 반길이 합 + 0.3 m)에서 선다 (줄 서기 · 따라가기 · 가로지르는 것 기다리기).
  정지한 로봇 · 차량에 3 s 넘게 막히면 Nav2 전역 재계획처럼 그 몸체를 장애물로 칠한 판에서 다시 계획한다
  (5 s 간격, 남은 길 + 30 m 이하일 때만 채택).
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import numpy as np

from layopt.sim import ensure_paths
from layopt.sim.layout import Pose
from layopt.sim.navmap import ACC, NavPath, OMEGA_MAX, V_MAX
from layopt.sim.workload import Task, uniform

ensure_paths()
from amr_behavior.battery_model import BatteryModel, BatteryParams  # noqa: E402
from amr_fleet.fleetsim.geometry import wrap  # noqa: E402
from amr_fleet.traffic_manager import (  # noqa: E402
    ST_CHARGING, ST_DOCKING, ST_ERROR, ST_IDLE, ST_LOADING, ST_MOVING,
)

# ---------------------------------------------------------------------- 단계
IDLE, TO_PICKUP, PERCEIVE, DOCK, LOAD, UNDOCK, TO_DROP, UNLOAD, RETURN = (
    'idle', 'to_pickup', 'perceive', 'dock', 'load', 'undock', 'to_drop', 'unload', 'return')
TO_CHARGER, TO_QUEUE, QUEUED, CHARGING = 'to_charger', 'to_queue', 'queued', 'charging'
DRIVE = frozenset({TO_PICKUP, TO_DROP, RETURN, TO_CHARGER, TO_QUEUE})
TIMED = frozenset({PERCEIVE, DOCK, LOAD, UNDOCK, UNLOAD})
STATUS = {IDLE: ST_IDLE, QUEUED: ST_IDLE, PERCEIVE: ST_DOCKING, DOCK: ST_DOCKING,
          UNDOCK: ST_DOCKING, LOAD: ST_LOADING, UNLOAD: ST_LOADING, CHARGING: ST_CHARGING}

# ---------------------------------------------------------------------- 상수 (근거: 모듈 머리말 · sim.md)
HALF_L, HALF_W = 0.30, 0.20
CIRCUM = math.hypot(HALF_L, HALF_W)
ROT_MIN = 0.785                 # rotate_to_heading_min_angle
YAW_TOL = 0.05                  # 목표 방위 허용
PERCEIVE_S = 3.3                # [측정] P5 PERCEIVING 중앙값
DOCK_S = (21.0, 24.5)           # [측정] P5 DOCKING 21.7–24.5 (적재) · 20.8–24.2 (하역)
UNDOCK_S = 15.8                 # [측정] P5 UNDOCKING 15.7–15.9
BATTERY_LOW = 20.0              # behavior.yaml battery_low_percent · accept_battery_min_percent
BATTERY_RESUME = 80.0           # battery_resume_percent
LOOK_M = 3.0                    # 몸체 검사 앞 거리
GAP_LAT = 0.30                  # 옆 안전 간격 (safety_node 0.30 m)
GAP_LON = 0.30                  # 앞 정지 간격
MAX_OFFSET = 0.95               # DWA max_path_offset_hard
WALL_GAP = 0.10
V_PASS = 0.5                    # 지나갈 때 속도 상한
PASS_ZONE = 1.5                 # 그 앞 이 거리 안에서 감속
QUEUE_M = 1.5                   # 내 정지 목표가 상대 너머 이 거리 안이면 줄 선다
OVERTAKE_V = 0.7                # 같은 방향 상대가 이보다 느리면 앞지를 수 있다
DETOUR_AFTER_S = 3.0
DETOUR_RETRY_S = 5.0
DETOUR_MAX_EXTRA_M = 30.0
SNAP_M = 1e-3


class Robot:
    """로봇 1 대 (sim 이 단계마다 step 을 부른다)."""

    def __init__(self, sim, rid: str, slot: Pose, soc: float):
        self.sim = sim
        self.id = rid
        self.slot = slot
        self.x, self.y, self.th = slot
        self.v = 0.0
        self.w = 0.0
        self.stage = IDLE
        self.task: Optional[Task] = None
        self.timer_until = 0.0
        self.charger: Optional[str] = None
        # 주행
        self.path: Optional[NavPath] = None
        self.s = 0.0
        self.goal: Optional[Pose] = None
        self.nav_phase = ''               # rot0 | follow | rot1
        self.align_goal = True
        # 교통 명령
        self.hold = False
        self.hold_reason = ''
        self.yielding = False
        self.yield_pose: Optional[Tuple[float, float, float]] = None
        self.saved_goal: Optional[Pose] = None
        self.keepout = None               # keepout 마스크 (지도 격자 bool) — 계획에 LETHAL
        self._km_obj = None               # 마지막으로 받은 KeepoutMask 객체
        self._ctx = 'pick'                # 도킹 맥락: pick | drop | charge
        # 막힘
        self.blocked_by: Optional[str] = None
        self.block_since: Optional[float] = None
        self.last_detour = -1e9
        # 측위 사건
        self.frozen_until = 0.0
        self.loc_failed = False
        # 배터리
        self.battery = BatteryModel(BatteryParams(initial_percent=soc))
        self.payload = 0.0
        self.st: Dict[str, float] = dict(
            distance_m=0.0, hold_s=0.0, hold_n=0, block_s=0.0, pass_s=0.0, detours=0, yields=0,
            keepouts=0, busy_s=0.0, charge_n=0, charge_s=0.0, queue_s=0.0, task_n=0)
        self._hold_prev = False

    # ------------------------------------------------------------------ 조회
    @property
    def soc(self) -> float:
        return self.battery.percent

    def status(self) -> int:
        if self.loc_failed:
            return ST_ERROR
        return STATUS.get(self.stage, ST_MOVING)

    def available(self, now: float) -> bool:
        """할당 후보 (IDLE · 잔량 ≥ 20 % · 측위 복구 중 아님)."""
        return (self.stage == IDLE and self.task is None and self.soc >= BATTERY_LOW
                and now >= self.frozen_until)

    def driving(self) -> bool:
        return self.stage in DRIVE and self.path is not None

    def plan_xy(self) -> Optional[np.ndarray]:
        """교통 관리자 관측의 plan (주행 · hold · 양보 중 경로)."""
        if self.stage in DRIVE and self.path is not None:
            return self.path.xy
        return None

    def priority(self) -> int:
        return self.task.priority if self.task is not None else -1

    # ------------------------------------------------------------------ 작업 · 충전 시작
    def start_task(self, now: float, task: Task) -> None:
        self.task = task
        self.st['task_n'] += 1
        self._nav(now, self.sim.layout.docks[task.pickup], TO_PICKUP)

    def start_charge(self, now: float) -> None:
        c = self.sim.chargers.claim(self)
        if c is not None:
            self.charger = c
            self.st['charge_n'] += 1
            self._nav(now, self.sim.layout.chargers[c], TO_CHARGER)
        else:
            q = self.sim.chargers.queue_pose(self)
            self._nav(now, q, TO_QUEUE, align=False)

    def _nav(self, now: float, goal: Pose, stage: str, align: bool = True,
             grid=None) -> None:
        self.stage = stage
        self.goal = goal
        self.align_goal = align
        self._plan_to(now, goal, grid)

    def _plan_to(self, now: float, goal: Pose, grid=None, cache: bool = True) -> bool:
        nm = self.sim.navmap
        if grid is None and self.keepout is not None:
            grid = nm.with_obstacles(nm.variant(self.id), lethal_mask=self.keepout)
        p = nm.plan((self.x, self.y), goal, self.id, grid=grid, cache=cache)
        if p is None and grid is not None:
            p = nm.plan((self.x, self.y), goal, self.id)        # 마스크로 막히면 원래 판
        if p is None:
            self.sim.count('plan_fail')
            p = NavPath([(self.x, self.y), (goal[0], goal[1])], goal[2])   # 직선 (거의 없음)
        self.path = p
        self.s = 0.0
        self.v = 0.0
        d = wrap(p.heading(0.0, self.th) - self.th)
        self.nav_phase = 'rot0' if abs(d) > ROT_MIN and p.length > 0.05 else 'follow'
        if self.nav_phase == 'follow' and p.length > 0.05:
            self.th = p.heading(0.0, self.th)
        return True

    # ------------------------------------------------------------------ 교통 명령 (TM tick 뒤)
    def apply_command(self, now: float, cmd) -> None:
        if cmd.keepout is not self._km_obj:
            self._set_keepout(now, cmd.keepout)
        if self.stage not in DRIVE:
            self.hold = False
            self.hold_reason = ''
            return
        if cmd.hold and cmd.yield_pose is not None:
            yp = tuple(float(v) for v in cmd.yield_pose)
            if not self.yielding or yp != self.yield_pose:
                self._start_yield(now, yp)
            self.hold, self.hold_reason = True, cmd.reason
            return
        if self.yielding and not cmd.hold:
            self._end_yield(now)
        self.hold = bool(cmd.hold)
        self.hold_reason = cmd.reason if cmd.hold else ''

    def _set_keepout(self, now: float, km) -> None:
        """keepout_mask 바뀜 → 전역 코스트맵 KeepoutFilter: 주행 중이면 지금 목표로 다시 계획."""
        self._km_obj = km
        if km is None:
            self.keepout = None
        else:
            self.st['keepouts'] += 1
            self.keepout = self.sim.mask_cells(km)
        if self.stage in DRIVE and self.goal is not None and not self.yielding:
            self._plan_to(now, self.goal, cache=False)

    def _start_yield(self, now: float, pose: Tuple[float, float, float]) -> None:
        if not self.yielding:
            self.saved_goal = self.goal
            self.st['yields'] += 1
        self.yielding = True
        self.yield_pose = pose
        nm = self.sim.navmap
        discs = [(o.x, o.y, CIRCUM + 0.2) for o in self.sim.robots if o is not self]
        grid = nm.with_obstacles(nm.variant(self.id), discs)
        p = nm.plan((self.x, self.y), (pose[0], pose[1], pose[2]), self.id, grid=grid)
        if p is None:
            p = nm.plan((self.x, self.y), (pose[0], pose[1], pose[2]), self.id)
        if p is not None:
            self.path, self.s, self.v = p, 0.0, 0.0
            d = wrap(p.heading(0.0, self.th) - self.th)
            self.nav_phase = 'rot0' if abs(d) > ROT_MIN and p.length > 0.05 else 'follow'
        self.align_goal = False

    def _end_yield(self, now: float) -> None:
        self.yielding = False
        self.yield_pose = None
        goal, self.saved_goal = self.saved_goal, None
        if goal is not None:
            self.goal = goal
            self.align_goal = self.stage != TO_QUEUE
            self._plan_to(now, goal)

    # ------------------------------------------------------------------ 한 단계
    def step(self, now: float, dt: float, k: int) -> None:
        sim = self.sim
        self.w = 0.0
        if now < self.frozen_until:                      # 측위 복구 (제자리 회전 · 재초기화 · FAILED)
            self.v = 0.0
            if self.stage in TIMED or self.stage == CHARGING:
                self.timer_until += dt
            self._battery(dt)
            return
        if self.loc_failed:
            self.loc_failed = False
        st = self.stage
        if st == IDLE:
            if self.task is None and self.soc < BATTERY_LOW and sim.charging_enabled:
                self.start_charge(now)
        elif st in TIMED:
            if now + dt >= self.timer_until - 1e-9:
                self._timer_done(now + dt)
        elif st == CHARGING:
            if self.soc >= BATTERY_RESUME:
                self._timed(now + dt, UNDOCK, UNDOCK_S)
        elif st == QUEUED:
            self.st['queue_s'] += dt
            c = sim.chargers.claim(self)
            if c is not None:
                self.charger = c
                self.st['charge_n'] += 1
                self._nav(now, sim.layout.chargers[c], TO_CHARGER)
        if self.stage in DRIVE and self.path is not None:
            self._drive(now, dt, k)
        else:
            self.v = 0.0
        if self.task is not None:
            self.st['busy_s'] += dt
        self._battery(dt)

    def _battery(self, dt: float) -> None:
        b = self.battery
        b.linear = self.v
        b.angular = self.w
        b.payload_mass = self.payload
        b.charging_enabled = self.stage == CHARGING
        b.step(dt)
        if self.stage == CHARGING:
            self.st['charge_s'] += dt

    def _timed(self, now: float, stage: str, dur: float) -> None:
        self.stage = stage
        self.timer_until = now + dur
        self.v = 0.0

    def _dock_s(self, tag: str) -> float:
        tid = self.task.task_id if self.task is not None else f'charge{self.st["charge_n"]}'
        return uniform(DOCK_S[0], DOCK_S[1], self.sim.seed, 'dock', self.id, tid, tag)

    def _arrive(self, now: float) -> None:
        st = self.stage
        self.path = None
        self.v = 0.0
        self.blocked_by = None
        self.block_since = None
        if st == TO_PICKUP:
            self._timed(now, PERCEIVE, PERCEIVE_S)
        elif st == TO_DROP:
            self._timed(now, DOCK, self._dock_s('drop'))
            self._ctx = 'drop'
        elif st == RETURN:
            self.stage = IDLE
        elif st == TO_CHARGER:
            self._timed(now, DOCK, self._dock_s('charge'))
            self._ctx = 'charge'
        elif st == TO_QUEUE:
            self.stage = QUEUED

    def _timer_done(self, now: float) -> None:
        st = self.stage
        ctx = getattr(self, '_ctx', 'pick')
        if st == PERCEIVE:
            self._ctx = 'pick'
            self._timed(now, DOCK, self._dock_s('pick'))
        elif st == DOCK:
            if ctx == 'pick':
                self._timed(now, LOAD, self.task.load_time)
            elif ctx == 'drop':
                self._timed(now, UNLOAD, self.task.load_time)
            else:
                self.stage = CHARGING
        elif st == LOAD:
            self.payload = self.task.mass
            self.sim.on_load(self, now)
            self._timed(now, UNDOCK, UNDOCK_S)
        elif st == UNLOAD:
            self.payload = 0.0
            self._timed(now, UNDOCK, UNDOCK_S)
        elif st == UNDOCK:
            if ctx == 'pick':
                self._nav(now, self.sim.layout.docks[self.task.dropoff],
                          TO_DROP)
            elif ctx == 'drop':
                self.sim.on_complete(self, now)
                self.task = None
                self._nav(now, self.slot, RETURN)
            else:
                self.sim.chargers.release(self)
                self.charger = None
                self._nav(now, self.slot, RETURN)

    # ------------------------------------------------------------------ 주행
    def _drive(self, now: float, dt: float, k: int) -> None:
        p = self.path
        moving_cmd = not self.hold or self.yielding
        if self.nav_phase == 'rot0':
            if not moving_cmd:
                self._hold_stat(dt)
                return
            self._rotate_to(p.heading(0.0, self.th), dt)
            if abs(wrap(p.heading(0.0, self.th) - self.th)) <= 1e-6:
                self.nav_phase = 'follow'
            self.v = 0.0
            return
        if self.nav_phase == 'rot1':
            if not moving_cmd:
                self._hold_stat(dt)
                return
            self._rotate_to(self.goal[2], dt)
            if abs(wrap(self.goal[2] - self.th)) <= YAW_TOL:
                self.th = self.goal[2]
                self._arrive(now + dt)
            return
        s = self.s
        stop = p.length
        if not moving_cmd:
            stop = min(stop, s + self.v * self.v / (2.0 * ACC))
        own_stop = stop
        bstop, vcap, blocker, bstatic = self._bodies(now, k, p, s, own_stop)
        if bstop < stop:
            stop = bstop
        dist = max(0.0, stop - s)
        adt = ACC * dt
        v_stop = -adt + math.sqrt(adt * adt + 2.0 * ACC * dist)
        v_new = max(0.0, min(self.v + adt, V_MAX, p.v_at(s), v_stop, vcap))
        s_new = s + v_new * dt
        if s_new >= stop - SNAP_M or dist <= SNAP_M:
            s_new = max(s, stop)
            v_end = 0.0
        else:
            v_end = v_new
        th_old = self.th
        self.s = s_new
        self.v = v_end
        nx, ny = p.point(s_new)
        self.st['distance_m'] += math.hypot(nx - self.x, ny - self.y)
        self.x, self.y = nx, ny
        if s_new < p.length - 1e-9:
            self.th = p.heading(s_new, self.th)
        self.w = abs(wrap(self.th - th_old)) / dt
        if vcap < V_MAX and v_end > 0:
            self.st['pass_s'] += dt
        if not moving_cmd:
            self._hold_stat(dt)
        else:
            self._hold_prev = False
        # 몸체 막힘 기록 · 전역 재계획
        if blocker is not None and v_end == 0.0 and bstop <= own_stop:
            self.st['block_s'] += dt
            if self.blocked_by != blocker:
                self.blocked_by, self.block_since = blocker, now
            if (bstatic and now - self.block_since >= DETOUR_AFTER_S
                    and now - self.last_detour >= DETOUR_RETRY_S and moving_cmd):
                self._detour(now, blocker)
        else:
            self.blocked_by, self.block_since = None, None
        if s_new >= p.length - 1e-9 and v_end == 0.0 and moving_cmd:
            if self.yielding:
                return                                   # 포켓에서 hold 가 풀릴 때까지
            if self.align_goal and self.goal is not None and \
                    abs(wrap(self.goal[2] - self.th)) > YAW_TOL:
                self.nav_phase = 'rot1'
            else:
                self._arrive(now + dt)

    def _hold_stat(self, dt: float) -> None:
        self.st['hold_s'] += dt
        if not self._hold_prev:
            self.st['hold_n'] += 1
        self._hold_prev = True

    def _rotate_to(self, target: float, dt: float) -> None:
        d = wrap(target - self.th)
        step = OMEGA_MAX * dt
        if abs(d) <= step:
            self.th = target
            self.w = abs(d) / dt
        else:
            self.th = wrap(self.th + math.copysign(step, d))
            self.w = OMEGA_MAX

    def _bodies(self, now: float, k: int, p: NavPath, s: float, own_stop: float):
        """경로 앞 몸체 → (정지 목표, 속도 상한, 막는 몸체 id, 그 몸체가 정지한 로봇·차량인가)."""
        sim = self.sim
        bx, bv, ids, hl, hw, kinds, held = sim.bodies
        if len(ids) <= 1:
            return math.inf, math.inf, None, False
        dx = bx[:, 0] - self.x
        dy = bx[:, 1] - self.y
        near = np.nonzero(dx * dx + dy * dy <= (LOOK_M + 1.5) ** 2)[0]
        stop, vcap, blocker, bstatic = math.inf, math.inf, None, False
        if len(near) <= 1:
            return stop, vcap, blocker, bstatic
        cum = p.cum_arr
        i0 = int(np.searchsorted(cum, s, side='right')) - 1
        i1 = int(np.searchsorted(cum, s + LOOK_M, side='right'))
        i0 = max(i0, 0)
        seg = p.xy[i0:i1 + 1]
        if len(seg) < 2:
            return stop, vcap, blocker, bstatic
        a0 = seg[:-1]
        ab = seg[1:] - a0
        ab2 = np.maximum((ab * ab).sum(axis=1), 1e-12)
        cands = []                       # [정지 목표, sb, 지나갈 수 있음(내 정지 목표 제외), j, 정지, along]
        for j in near.tolist():
            if ids[j] == self.id:
                continue
            ap = bx[j] - a0                                   # 선분 투영 (경로 점 간격과 무관)
            tt = np.clip((ap * ab).sum(axis=1) / ab2, 0.0, 1.0)
            dd = ap - ab * tt[:, None]
            d2 = (dd * dd).sum(axis=1)
            m = int(np.argmin(d2))
            lat = math.sqrt(float(d2[m]))
            sb = float(cum[i0 + m]) + float(tt[m]) * math.sqrt(float(ab2[m]))
            if sb <= s - 1.0:
                continue
            thp = math.atan2(ab[m, 1], ab[m, 0])
            px, py = a0[m, 0] + tt[m] * ab[m, 0], a0[m, 1] + tt[m] * ab[m, 1]
            vx, vy = bv[j, 0], bv[j, 1]
            spd = math.hypot(vx, vy)
            if kinds[j] == 'robot':
                thb = sim.robot_th[j]
            else:
                thb = math.atan2(vy, vx) if spd > 0.05 else thp
            dpsi = thb - thp
            sn, cs = abs(math.sin(dpsi)), abs(math.cos(dpsi))
            across = hl[j] * sn + hw[j] * cs
            along = hl[j] * cs + hw[j] * sn
            thr = HALF_W + across + GAP_LAT
            if lat >= thr:
                continue
            if spd < 0.05:
                cls = 'static'
            else:
                ang = abs(wrap(math.atan2(vy, vx) - thp))
                cls = 'same' if ang < math.pi / 4 else ('opp' if ang > 3 * math.pi / 4
                                                        else 'cross')
            need = thr - lat
            room = need <= MAX_OFFSET and \
                sim.navmap.clearance_at(px, py) >= need + HALF_W + WALL_GAP
            # 교통 hold 로 선 로봇 뒤에 같은 방향으로 오면 줄 선다 (같은 구역을 기다린다). 마주 보는 경우는
            # 지나갈 수 있다 — P5 Gazebo 에서 도크 접근 대기 로봇 옆을 0.8–0.9 m 로 지나간 기록 (sim.md)
            queue_behind = held[j] and abs(wrap(thb - thp)) < math.pi / 2
            passable = (room and cls != 'cross' and not queue_behind
                        and (cls != 'same' or spd < OVERTAKE_V))
            if sb <= s + 0.05:                               # 옆 · 바로 뒤: 지나가는 중 — 감속만
                vcap = min(vcap, V_PASS)
                continue
            cands.append([sb - (HALF_L + along + GAP_LON), sb, passable, j, spd < 0.05, along])
        if not cands:
            return stop, vcap, blocker, bstatic
        # 내 정지 목표(경로 끝 · hold · 지나갈 수 없는 몸체 앞)가 지나가려는 몸체 너머 QUEUE_M 안이면 그 뒤에
        # 줄 선다 — 막힌 줄을 앞지르지 않게 (막는 몸체가 늘면 다시 본다)
        combined = min([own_stop] + [c[0] for c in cands if not c[2]])
        changed = True
        while changed:
            changed = False
            for c in cands:
                if c[2] and combined <= c[1] + QUEUE_M:
                    c[2] = False
                    combined = min(combined, c[0])
                    changed = True
        for st, sb, passable, j, still, along in cands:
            if passable:
                if sb - s < PASS_ZONE + along:
                    vcap = min(vcap, V_PASS)
            elif st < stop:
                stop, blocker = st, ids[j]
                bstatic = still and kinds[j] != 'human'
        return stop, vcap, blocker, bstatic

    def _detour(self, now: float, blocker: str) -> None:
        """정지 몸체를 장애물로 칠하고 다시 계획 (Nav2 전역 코스트맵 obstacle_layer 근사)."""
        self.last_detour = now
        sim = self.sim
        j = sim.body_index(blocker)
        if j is None or self.goal is None:
            return
        bx, _, _, hl, hw, _, _ = sim.bodies
        r = math.hypot(hl[j], hw[j]) + HALF_W + 0.05
        nm = sim.navmap
        base = nm.variant(self.id)
        grid = nm.with_obstacles(base, [(float(bx[j, 0]), float(bx[j, 1]), r)],
                                 lethal_mask=self.keepout)
        goal = self.yield_pose if self.yielding else self.goal
        if goal is None:
            return
        q = nm.plan((self.x, self.y), (goal[0], goal[1], goal[2] if len(goal) > 2 else 0.0),
                    self.id, grid=grid)
        if q is None:
            return
        remaining = self.path.length - self.s
        if q.length > remaining + DETOUR_MAX_EXTRA_M:
            return
        self.path, self.s = q, 0.0
        d = wrap(q.heading(0.0, self.th) - self.th)
        self.nav_phase = 'rot0' if abs(d) > ROT_MIN and q.length > 0.05 else 'follow'
        self.st['detours'] += 1
        sim.count('detour')


__all__ = ['BATTERY_LOW', 'BATTERY_RESUME', 'CHARGING', 'DOCK', 'DRIVE', 'IDLE', 'LOAD',
           'PERCEIVE', 'QUEUED', 'RETURN', 'Robot', 'TO_CHARGER', 'TO_DROP', 'TO_PICKUP',
           'TO_QUEUE', 'UNDOCK', 'UNLOAD']
