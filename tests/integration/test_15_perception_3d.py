"""
시나리오 15: AI 기반 인지 — 검출 · 3D 변환 · 마커 (명세 4.6 "AI 기반 인지 시스템").

스택(system 프로필): Gazebo + localization + navigation + perception (yolo_node → object_localizer_node
→ detection_marker_node). 스폰 = 월드 원점 (명시). navigation 을 켜는 이유는 주행이 아니라 순간 이동 뒤
위치 추정 복구(kidnap_monitor → behavior_server 제자리 회전)가 있어야 map TF 가 다시 맞기 때문이다 —
이 시나리오는 navigate_to_pose 를 쓰지 않는다.

지면 진실은 제품 코드를 그대로 쓴다 (amr_perception/world_objects.py): parse_world 가 월드 SDF 에서 라벨
대상 3D 박스(include 모델 = 화물, 인라인 판 = 표지판)와 actor 궤적(작업자)을 뽑고, label_objects 가 깊이
영상으로 가림·잘림을 검증한 2D 라벨을 만든다. 시점도 그 지면 진실에서 계산한다: 클래스마다 대상 물체를
고르고, 구조물 단면(worldmap.footprints)에서 떨어져 있으면서 시선이 트인 자리를 찾아 로봇을 순간 이동시킨다
(10 과 같은 kidnap 감지 + 안정 대기). 복구 회전으로 방위가 흐트러지므로 GT 요각을 보고 제자리 회전으로
목표 방위를 다시 맞춘다. 작업자 시점은 궤적을 보고 작업자가 대상 지점에 다가올 때까지 기다린 뒤 모은다.

판정 — 명세 4.6 에 수치·문장이 있는 것만
  검출 클래스   perception/detected_objects 에 나온 출력 클래스 ≥ 3 종
                ("화물, 사람, 표지판 등 3종 이상의 객체를 인식해야 한다")
  추론 속도     배포 설정(perception.yaml)으로 만든 YoloDetector 로 실제 카메라 프레임을 추론한 프레임별
                지연 중앙값 → FPS. GPU 경로 ≥ 30, CPU 경로 ≥ 10
                ("추론 속도는 CPU 기준 10 FPS 이상(또는 GPU 가속 시 30 FPS 이상) 유지되어야 한다").
                토픽 발행률로 재지 않는다: 카메라가 sim 30 Hz 라 벽시계로는 30×RTF Hz 이고, 그 값을 FPS 로
                적으면 시뮬레이터 속도를 추론 속도로 보고하게 된다 (perception.md §8.2 의 Gazebo 종단
                1.4~7.1 Hz). 발행률·처리율은 따로 측정만 한다.
  커스텀 메시지  perception/detected_objects 의 형이 amr_msgs/msg/DetectedObjectArray 이고 필드가 채워짐
                ("인식된 객체 정보를 커스텀 메시지 타입으로 발행하고")
  map 프레임    모든 pose_3d.header.frame_id == 'map'
                ("2D 픽셀 좌표를 Depth 정보와 결합하여 3D 공간 좌표(Map frame)로 변환한다")
  마커          perception/markers 가 객체마다 CUBE + TEXT 를 map 프레임으로, pose_3d 위치에 발행
                ("RViz2에 마커로 시각화", "3D 위치가 지도상에 마커로 표시되어야 한다"). 이 서버에는
                DISPLAY 가 없으므로 RViz2 화면이 아니라 토픽으로 판정한다.

측정만 — 명세 4.6 에 수치가 없다 (임계값을 지어내지 않는다)
  GT 대비 3D 위치 오차, pose_3d → 픽셀 재투영 잔차(핀홀 역투영의 역연산), 클래스별 재현율·정밀도,
  detections_2d/detected_objects/markers 발행률·RTF·처리율. 유일한 가드는 3D xy 오차 중앙값 ≤ 1.0 m 이고
  이것은 **하네스 선택(명세 아님)** — 변환 배관이 붙어 있는지만 보는 느슨한 값이다 (perception.md §8.4
  실측 중앙값 ≈ 0.09 m). GT 매칭 IoU 0.5·필수 GT 규칙(짧은 변 ≥ 16 px, 거리 ≤ 10 m)도 COCO 관례이자
  하네스 선택이며 판정에 쓰지 않는다.
로그 perception.csv · views.csv · inference.csv.
"""

import math
import os
import time
from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple

from amr_itest import actions, cases, catalog, config, gz, metrics, rates, worldmap
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

CTX = Context(catalog.get(15))

SPAWN = (0.0, 0.0, 0.0)          # 월드 원점 (랙 B-C 통로 한가운데) — 모든 system 시나리오처럼 명시
MAP_FRAME = 'map'
CUSTOM_MSG = 'amr_msgs/msg/DetectedObjectArray'
OBJECTS_TOPIC = 'perception/detected_objects'
MARKERS_TOPIC = 'perception/markers'
DETECTIONS_2D_TOPIC = 'perception/detections_2d'
OUTPUT_CLASSES = ('box', 'person', 'sign')   # classes.yaml (명세 "화물, 사람, 표지판")
MIN_CLASSES = 3                  # 명세 4.6 "3종 이상"
GPU_FPS = 30.0                   # 명세 4.6 "GPU 가속 시 30 FPS 이상"
CPU_FPS = 10.0                   # 명세 4.6 "CPU 기준 10 FPS 이상"
LIFECYCLE = ('/lifecycle_manager_map', 'lifecycle_manager_localization',
             'lifecycle_manager_navigation')

# --- 시점 계산 (지면 진실에서) ---
VIEW_ORDER = ('person', 'sign', 'box')   # 얻기 어려운 클래스부터 (러너 상한에 잘리면 뒤가 빠진다)
VIEW_DIST = {'box': 3.5, 'sign': 3.5, 'person': 4.0}    # [m] 대상까지 거리
VIEW_WINDOW_S = {'box': 12.0, 'sign': 12.0, 'person': 25.0}   # [s, sim] 수집 창
ROBOT_CLEARANCE = 0.55           # [m] 로봇 외접원 0.361 + 여유 (순간 이동 자리가 구조물에 끼지 않게)
SIGHT_PAD = 0.05                 # [m] 시선 판정 여유
SIGHT_STEP = 0.1                 # [m] 시선 표본 간격 (벽 두께 0.2 m 보다 촘촘히)
SIGHT_STOP_SHORT = 0.35          # [m] 대상 앞 이만큼은 보지 않는다 (판이 붙은 랙 면)
BEARINGS = 24                    # 대상 둘레 후보 방위 수
PERSON_BOX = (0.7, 0.7, 1.9)     # dataset_capture.py 와 같은 작업자 라벨 박스
ACTOR_NEAR_M = 2.5               # [m] 작업자가 대상 지점에 이만큼 들어오면 수집 시작
ACTOR_WAIT_CAP_S = 90.0          # [s, sim] 작업자 대기 상한

# --- 수집·판정 ---
MAX_FRAMES_PER_VIEW = 60
MIN_FRAMES_PER_VIEW = 8
HIT_FRAMES_PER_VIEW = 8          # 시점 클래스의 필수 GT 가 든 프레임이 이만큼 모이면 창을 일찍 닫는다
GT_MATCH_NS = 30_000_000         # [ns] 깊이 스탬프 ↔ ground_truth/odom 표본 허용 차
IOU_MATCH = 0.5                  # COCO 관례 (하네스 선택 — 명세 아님, 측정에만 쓴다)
GT_MIN_PX = 16.0                 # 필수 GT 짧은 변 [px] (하네스 선택 — 명세 아님)
GT_MAX_DIST = 10.0               # 필수 GT 거리 [m] = depth 최대 거리 (하네스 선택 — 명세 아님)
GT_LABEL_RANGE = 12.0            # [m] 라벨 대상 거리 상한 (GT_MAX_DIST + 여유, 프레임당 계산량)
# 하네스 선택(명세 아님): 명세 4.6 에는 3D 위치 오차 수치가 없다. 변환 배관이 아예 어긋났는지만 보는
# 느슨한 가드다 (perception.md §8.4 실측 중앙값 0.082~0.095 m 의 10 배).
GT_ERR_GUARD_M = 1.0
MARKER_EPS = 1e-3                # [m] 마커 위치 == pose_3d 수치 비교 (임계값이 아니라 부동소수 여유)
RATE_WINDOW_S = 10.0             # [s, sim] 발행률 측정 창
INFER_FRAMES = 30                # 추론 지연 표본 수

# --- 시간 상한 ---
STARTUP_WALL_S = 420.0           # [s] Gazebo + 전체 스택 + YOLO 가중치 적재
KIDNAP_DETECT_S = 6.0            # [s] kidnap_monitor 감지 창 (10 과 같은 값)
SETTLE_S = 4.0                   # [s] localization/lost=false 유지
RELOCALIZE_MAX_S = 90.0          # [s] 순간 이동 뒤 위치 추정 안정 상한
FACE_MAX_S = 40.0                # [s] 목표 방위 맞추기 상한
YAW_TOL = math.radians(3.0)      # [rad] 방위 허용 오차 (시야 87° 안에 대상이 들어오면 충분)

PERCEPTION_COLUMNS = [
    'view', 't', 'class_name', 'confidence', 'bbox_x', 'bbox_y', 'bbox_width', 'bbox_height',
    'pose_3d_x', 'pose_3d_y', 'pose_3d_z', 'gt_name', 'gt_x', 'gt_y', 'gt_z', 'err_m',
    'err_xy_m', 'reproj_px', 'distance_m', 'frame_id', 'matched']
VIEW_COLUMNS = ['view', 'target_class', 'target', 'x', 'y', 'yaw', 'frames', 'objects',
                'gt_required', 'matched', 'classes']


class Viewpoint(NamedTuple):
    """한 시점: 대상 물체와 그것을 바라보는 로봇 자세."""

    cls: str
    target_name: str
    target: Tuple[float, float]
    x: float
    y: float
    yaw: float
    window_s: float
    track: object = None          # person 시점이면 wo.ActorTrack (도착을 기다린다)


# ---------------------------------------------------------------- 지면 진실 · 시점

def stamp_key(stamp) -> int:
    """header.stamp → ns 정수 키 (깊이·detected_objects·markers 는 같은 스탬프를 쓴다)."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def point_free(shapes: Sequence, x: float, y: float, pad: float) -> bool:
    """(x, y) 가 모든 구조물 단면에서 pad 이상 떨어져 있는가."""
    px, py = np.array([float(x)]), np.array([float(y)])
    return not any(bool(s.contains(px, py, pad)[0]) for s in shapes)


def sight_clear(shapes: Sequence, x0: float, y0: float, x1: float, y1: float) -> bool:
    """(x0, y0) → (x1, y1) 시선이 대상 SIGHT_STOP_SHORT 앞까지 구조물에 막히지 않는가."""
    span = math.hypot(x1 - x0, y1 - y0)
    usable = span - SIGHT_STOP_SHORT
    if usable <= 0.0 or span <= 0.0:
        return False
    ux, uy = (x1 - x0) / span, (y1 - y0) / span
    n = max(int(usable / SIGHT_STEP), 1)
    return all(point_free(shapes, x0 + ux * usable * i / n, y0 + uy * usable * i / n, SIGHT_PAD)
               for i in range(n + 1))


def observation_pose(shapes: Sequence, tx: float, ty: float,
                     distance: float) -> Optional[Tuple[float, float, float]]:
    """대상 (tx, ty) 를 distance [m] 에서 바라보는 자유·시선 확보 자세 (없으면 None)."""
    for i in range(BEARINGS):
        theta = 2.0 * math.pi * i / BEARINGS
        x, y = tx + distance * math.cos(theta), ty + distance * math.sin(theta)
        if not point_free(shapes, x, y, ROBOT_CLEARANCE):
            continue
        if not sight_clear(shapes, x, y, tx, ty):
            continue
        return x, y, math.atan2(ty - y, tx - x)
    return None


def class_targets(statics: Sequence, actors: Sequence, cls: str) -> List[tuple]:
    """클래스별 대상 후보 [(이름, (x, y), 궤적 또는 None)] — 결정적 순서."""
    if cls == 'person':
        out = []
        for a in sorted(actors, key=lambda t: t.name):
            xy = np.asarray(a.xyz, dtype=float)[:, :2]
            if len(xy) < 2:
                continue
            i = int(np.argmax(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
            mid = 0.5 * (xy[i] + xy[i + 1])       # 가장 긴 구간의 중점 = 작업자가 지나는 자리
            out.append((a.name, (float(mid[0]), float(mid[1])), a))
        return out
    objs = [o for o in statics if o.class_name == cls]
    # 큰 물체부터 (멀리서도 픽셀이 충분하다), 같은 크기면 이름 순 — 시드 없이 결정적
    objs.sort(key=lambda o: (-float(np.prod(np.asarray(o.size, dtype=float))), o.name))
    return [(o.name, (float(o.center[0]), float(o.center[1])), None) for o in objs]


def build_viewpoints(statics: Sequence, actors: Sequence, shapes: Sequence) -> List[Viewpoint]:
    """VIEW_ORDER 순서로 클래스마다 시점 하나씩 (자리를 못 찾은 클래스는 빠진다)."""
    views: List[Viewpoint] = []
    for cls in VIEW_ORDER:
        for name, (tx, ty), track in class_targets(statics, actors, cls):
            pose = observation_pose(shapes, tx, ty, VIEW_DIST[cls])
            if pose is None:
                continue
            views.append(Viewpoint(cls, name, (tx, ty), pose[0], pose[1], pose[2],
                                   VIEW_WINDOW_S[cls], track))
            break
    return views


def gt_entries(labels: Sequence) -> List[tuple]:
    """
    깊이 검증 라벨 → (클래스, bbox, 필수 여부, 이름, 거리). 출력 클래스만.

    필수 GT = 재현율의 분모 (보이고·잘리지 않고·충분히 크고 깊이 범위 안). 나머지 채택·가림 라벨은
    무시 GT 라 맞혀도 FP 가 아니다. scripts/yolo_live_eval.py 와 같은 규칙이며 전부 하네스 선택이다
    (명세 4.6 에는 재현율 수치가 없어 측정만 한다).
    """
    out = []
    for lab in labels:
        if lab.class_name not in OUTPUT_CLASSES:
            continue
        side = min(lab.bbox[2] - lab.bbox[0], lab.bbox[3] - lab.bbox[1])
        if lab.kept:
            required = (lab.visible >= 0.5 and lab.truncated <= 0.3 and side >= GT_MIN_PX
                        and lab.distance <= GT_MAX_DIST)
        elif lab.reason in ('occluded', 'truncated', 'small') and lab.pixels > 0:
            required = False
        else:
            continue
        out.append((lab.class_name, lab.bbox, required, lab.name, lab.distance))
    return out


def object_bbox(obj) -> Tuple[float, float, float, float]:
    """amr_msgs/DetectedObject 의 픽셀 bbox [x1, y1, x2, y2]."""
    return (float(obj.bbox_x), float(obj.bbox_y), float(obj.bbox_x + obj.bbox_width),
            float(obj.bbox_y + obj.bbox_height))


def marker_problems(arr: MarkerArray, objs) -> List[str]:
    """마커 배열이 객체마다 map 프레임 CUBE + TEXT 를 pose_3d 위치에 담고 있는가 (문제 목록)."""
    out: List[str] = []
    markers = list(arr.markers)
    if not markers or markers[0].action != Marker.DELETEALL:
        out.append('DELETEALL 로 시작하지 않는다')
    body = [m for m in markers if m.action == Marker.ADD]
    cubes = [m for m in body if m.type == Marker.CUBE]
    texts = [m for m in body if m.type == Marker.TEXT_VIEW_FACING]
    n = len(objs.objects)
    if len(cubes) != n or len(texts) != n:
        out.append(f'객체 {n} 개에 CUBE {len(cubes)} · TEXT {len(texts)}')
    frames = sorted({m.header.frame_id for m in body})
    if frames and frames != [MAP_FRAME]:
        out.append(f'마커 프레임 {frames} (map 이 아니다)')
    for obj, cube, text in zip(objs.objects, cubes, texts):
        px = obj.pose_3d.pose.position.x
        py = obj.pose_3d.pose.position.y
        placed = all(abs(m.pose.position.x - px) <= MARKER_EPS
                     and abs(m.pose.position.y - py) <= MARKER_EPS for m in (cube, text))
        if not placed:
            out.append(f'{obj.class_name} 마커가 pose_3d 위치에 있지 않다')
        if not text.text:
            out.append(f'{obj.class_name} 마커 텍스트가 비어 있다')
    return out


def nearest_message(messages: Sequence[tuple], key: int, tol_ns: int):
    """(수신 시각, 메시지) 목록에서 header.stamp 가 key 에 가장 가까운 메시지 (tol 밖이면 None)."""
    best, best_gap = None, None
    for _, msg in messages:
        gap = abs(stamp_key(msg.header.stamp) - key)
        if best_gap is None or gap < best_gap:
            best, best_gap = msg, gap
    return best if best_gap is not None and best_gap <= tol_ns else None


def next_frame(depth_rec, obj_rec, marker_rec, seen: set):
    """아직 안 본 스탬프 중 가장 최신의 (키, 깊이, detected_objects, markers) — 없으면 None."""
    depths = {stamp_key(m.header.stamp): m for _, m in depth_rec.messages()}
    objs = {stamp_key(m.header.stamp): m for _, m in obj_rec.messages()}
    keys = sorted(k for k in set(depths) & set(objs) if k not in seen)
    if not keys:
        return None
    key = keys[-1]                 # 오래된 프레임은 곧 밀려난다 — 최신부터 쓴다
    seen.add(key)
    marks = None
    for _, m in marker_rec.messages():
        if m.markers and stamp_key(m.markers[0].header.stamp) == key:
            marks = m
    return key, depths[key], objs[key], marks


# ---------------------------------------------------------------- launch

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements(use_behavior=False)
                + [req.executable('amr_perception', 'yolo_node', 'YOLOv8 2D 검출'),
                   req.executable('amr_perception', 'object_localizer_node',
                                  '2D + Depth → 3D (map)'),
                   req.executable('amr_perception', 'detection_marker_node', 'RViz 마커'),
                   req.config('amr_perception', 'perception.yaml', '배포 추론 설정'),
                   req.config('amr_perception', 'classes.yaml', '출력 클래스 3종'),
                   req.package('amr_msgs'),
                   req.module('ultralytics', 'YOLOv8 추론'),
                   req.module('cv_bridge', '이미지 → OpenCV')], 'perception')
    stack = Stack(CTX, *CTX.select())
    stack.system(use_localization=True, use_navigation=True, use_perception=True,
                 use_behavior=False, pose=SPAWN)
    return stack.launch_description(), {'stack': stack}


class TestPerception(cases.ProbeCase):
    """검출 3종 · 커스텀 메시지 · map 프레임 3D · 마커 · 추론 속도."""

    CTX = CTX

    def test_05_ready(self) -> None:
        """기동 게이트 (launch_testing_ros WaitForTopics): GT 오도메트리가 흐른다."""
        self.ready_gate([('ground_truth/odom', Odometry)], STARTUP_WALL_S)

    # ------------------------------------------------------------ 시점 이동

    def _settled(self, lost) -> bool:
        """localization/lost 가 false 로 SETTLE_S (sim) 동안 이어질 때까지 (복구 회전 포함)."""
        state = {'since': None}

        def ok() -> bool:
            msg = lost.last()
            now = self.probe.now()
            if msg is None or msg.data:
                state['since'] = None
                return False
            state['since'] = now if state['since'] is None else state['since']
            return now - state['since'] >= SETTLE_S
        return self.probe.wait_until(ok, self.timeout(RELOCALIZE_MAX_S), 0.1)

    def _teleport(self, world: str, pose, lost, what: str) -> None:
        """Gazebo set_pose + kidnap 감지·안정 대기 (10 과 같은 절차)."""
        ok, out = gz.set_pose(world, self.settings.robot, *pose)
        self.assertTrue(ok, f'{what}: set_pose 실패: {out}')
        # 순간 이동은 위치 추정에는 납치다: kidnap_monitor_node 가 감지하면 재초기화 + 제자리 회전
        # (behavior_server spin → cmd_vel_nav) 을 하므로 끝나기 전에 재면 map TF 가 아직 어긋나 있다.
        self.assertTrue(self.probe.sleep_ros(KIDNAP_DETECT_S, self.timeout(120.0)))
        self.assertTrue(self._settled(lost), f'{what}: 순간 이동 뒤 위치 추정이 안정되지 않음')

    def _face(self, drive_topic: str, gt, yaw_goal: float) -> float:
        """제자리 회전으로 GT 방위를 yaw_goal 에 맞춘다 (복구 회전이 방위를 바꿔 놓는다). 반환: 남은 오차."""
        deadline = time.monotonic() + self.timeout(FACE_MAX_S)
        err = math.pi
        while time.monotonic() < deadline:
            msg = gt.last()
            if msg is None:
                time.sleep(0.05)
                continue
            err = metrics.wrap(yaw_goal - metrics.sample_from_odom(msg).yaw)
            if abs(err) <= YAW_TOL:
                break
            rate = math.copysign(min(max(abs(err), 0.2), 0.6), err)
            actions.drive_for(self.probe, drive_topic, 0.0, rate, 0.2, self.timeout(10.0))
        actions.drive_for(self.probe, drive_topic, 0.0, 0.0, 0.5, self.timeout(20.0))
        actions.wait_rest(self.probe, gt, hold=0.5, timeout=self.timeout(30.0))
        msg = gt.last()
        if msg is not None:
            err = metrics.wrap(yaw_goal - metrics.sample_from_odom(msg).yaw)
        return abs(err)

    def _wait_for_actor(self, view: Viewpoint) -> bool:
        """
        작업자가 대상 지점 ACTOR_NEAR_M 안으로 들어올 때까지 (궤적 해석 — 시뮬레이터 질의 없음).

        창은 sim 시각으로 센다: actor 는 sim time 으로 걷는데 벽시계로 기다리면 RTF 만큼 짧아져
        (RTF 0.3 이면 90 wall s = 27 sim s) 궤적 주기를 못 채운다.
        """
        if view.track is None:
            return True
        tx, ty = view.target
        sim_end = self.probe.now() + ACTOR_WAIT_CAP_S

        def near() -> bool:
            if self.probe.now() >= sim_end:
                return True                     # sim 창 소진 — 도착 여부는 아래에서 다시 본다
            pos = view.track.pose_at(self.probe.now())[0]
            return math.hypot(float(pos[0]) - tx, float(pos[1]) - ty) <= ACTOR_NEAR_M

        self.probe.wait_until(near, self.timeout(ACTOR_WAIT_CAP_S * 6.0), 0.2)
        pos = view.track.pose_at(self.probe.now())[0]
        return math.hypot(float(pos[0]) - tx, float(pos[1]) - ty) <= ACTOR_NEAR_M

    # ------------------------------------------------------------ 프레임 처리

    def _reproject(self, env: dict, key: int,
                   point) -> Optional[Tuple[float, float]]:
        """
        pose_3d(map) → 픽셀 재투영: 핀홀 K 와 map→optical TF 로 되돌린다 (TF 가 없으면 None).

        측정만 한다 (명세 4.6 에 잔차 수치가 없다). 노드가 쓴 것과 같은 TF 를 쓰므로 위치 추정 오차는
        상쇄되고 핀홀 역투영의 왕복 일관성만 남는다. 표면→중심 보정은 시선 방향이라 픽셀을 바꾸지 않는다.
        """
        from amr_perception.transforms import transform_from_msg
        from rclpy.time import Time as RclTime
        import tf2_ros
        try:
            tf = env['tf'].lookup_transform(env['optical'], MAP_FRAME, RclTime(nanoseconds=key))
        except tf2_ros.TransformException:
            return None
        p = transform_from_msg(tf.transform).apply(point)
        if p[2] <= 1e-3:
            return None
        k = env['kin']
        return k.fx * p[0] / p[2] + k.cx, k.fy * p[1] / p[2] + k.cy

    def _process(self, index: int, view: Viewpoint, env: dict, acc: dict, key: int,
                 depth_msg, obj_msg, marker_msg) -> Optional[bool]:
        """프레임 하나를 지면 진실과 맞춰 누적한다. 반환: 시점 클래스의 필수 GT 가 있었나 (못 쓰면 None)."""
        from amr_perception import world_objects as wo
        from amr_perception.object_localizer_node import depth_to_meters
        from amr_perception.transforms import quat_to_matrix
        depth_m = depth_to_meters(depth_msg)
        if depth_m is None:
            return None
        odom = nearest_message(env['gt'].messages(), key, GT_MATCH_NS)
        if odom is None:
            return None
        t = key * 1e-9
        pos, quat = odom.pose.pose.position, odom.pose.pose.orientation
        t_wb = wo.homogeneous(quat_to_matrix((quat.x, quat.y, quat.z, quat.w)),
                              (pos.x, pos.y, pos.z))
        r_cw, t_cw = wo.world_to_camera(t_wb, env['base_to_optical'])
        objects = list(env['statics']) + wo.actor_objects(env['actors'], t, PERSON_BOX)
        labels = wo.label_objects(objects, r_cw, t_cw, env['kin'], depth_m, env['label_params'])
        gts = gt_entries(labels)
        dets = [(o.class_name, object_bbox(o), float(o.confidence)) for o in obj_msg.objects]
        gt_match, det_match = wo.match_detections([g[:3] for g in gts], dets, IOU_MATCH)
        by_name = {o.name: o for o in objects}

        acc['frames'] += 1
        acc['header_frames'].add(obj_msg.header.frame_id)
        if marker_msg is not None:
            acc['marker_frames'] += 1
            acc['marker_problems'] += marker_problems(marker_msg, obj_msg)
            if obj_msg.objects:
                acc['marker_object_frames'] += 1
        for (cls, _, required, _, _), m in zip(gts, gt_match):
            if not required:
                continue
            acc['gt'][cls] = acc['gt'].get(cls, 0) + 1
            if m >= 0:
                acc['tp'][cls] = acc['tp'].get(cls, 0) + 1
        for j, obj in enumerate(obj_msg.objects):
            acc['objects'] += 1
            acc['classes'].add(obj.class_name)
            acc['pose_frames'].add(obj.pose_3d.header.frame_id)
            if (obj.class_name in OUTPUT_CLASSES and obj.bbox_width > 0 and obj.bbox_height > 0
                    and obj.confidence > 0.0 and obj.pose_3d.header.frame_id):
                acc['fields_ok'] = True
            gi = det_match[j]
            if gi < 0 and obj.class_name in OUTPUT_CLASSES:
                acc['fp'][obj.class_name] = acc['fp'].get(obj.class_name, 0) + 1
            gt_obj = by_name.get(gts[gi][3]) if gi >= 0 else None
            p = obj.pose_3d.pose.position
            gx = gy = gz = err = err_xy = math.nan
            if gt_obj is not None:
                gx, gy, gz = (float(v) for v in gt_obj.center)
                err = float(math.dist((p.x, p.y, p.z), (gx, gy, gz)))
                err_xy = float(math.hypot(p.x - gx, p.y - gy))
                acc['err_xy'].append(err_xy)
                acc['err_by_class'].setdefault(obj.class_name, []).append(err_xy)
            uv = self._reproject(env, key, (p.x, p.y, p.z))
            reproj = math.nan
            if uv is not None:
                cx = obj.bbox_x + 0.5 * obj.bbox_width
                cy = obj.bbox_y + 0.5 * obj.bbox_height
                reproj = float(math.hypot(uv[0] - cx, uv[1] - cy))
                acc['reproj'].append(reproj)
            acc['rows'].append([
                index, round(t, 4), obj.class_name, round(float(obj.confidence), 4),
                int(obj.bbox_x), int(obj.bbox_y), int(obj.bbox_width), int(obj.bbox_height),
                round(p.x, 4), round(p.y, 4), round(p.z, 4),
                gt_obj.name if gt_obj is not None else '', cases.fmt(gx), cases.fmt(gy),
                cases.fmt(gz), cases.fmt(err), cases.fmt(err_xy), cases.fmt(reproj, 2),
                round(float(obj.distance), 4), obj.pose_3d.header.frame_id, int(gi >= 0)])
        return any(g[0] == view.cls and g[2] for g in gts)

    def _collect(self, index: int, view: Viewpoint, env: dict, acc: dict) -> dict:
        """시점 하나에서 프레임을 모은다 (창 또는 충분한 표본까지)."""
        seen: set = set()
        frames = hits = 0
        sim_end = self.probe.now() + view.window_s
        wall_end = time.monotonic() + self.timeout(view.window_s * 8.0 + 120.0)
        before = len(acc['rows'])
        gt_before = acc['gt'].get(view.cls, 0)
        while (self.probe.now() < sim_end and time.monotonic() < wall_end
               and frames < MAX_FRAMES_PER_VIEW):
            if hits >= HIT_FRAMES_PER_VIEW and frames >= MIN_FRAMES_PER_VIEW:
                break
            got = next_frame(env['depth'], env['objs'], env['marks'], seen)
            if got is None:
                time.sleep(0.05)
                continue
            hit = self._process(index, view, env, acc, *got)
            if hit is None:
                continue
            frames += 1
            hits += int(hit)
        rows = acc['rows'][before:]
        return {'frames': frames, 'hits': hits, 'objects': len(rows),
                'gt_required': acc['gt'].get(view.cls, 0) - gt_before,
                'matched': sum(r[-1] for r in rows),
                'classes': sorted({r[2] for r in rows})}

    # ------------------------------------------------------------ 본 시험

    def test_10_pipeline(self, stack) -> None:
        """검출 3종 · 커스텀 메시지 · map 프레임 3D · 객체마다 마커 (명세 4.6)."""
        from amr_msgs.msg import DetectedObjectArray
        from amr_perception import world_objects as wo
        share = req.share_dir('amr_simulation')
        world_sdf = share / 'worlds' / self.settings.world
        world = self.settings.world.rsplit('.', 1)[0]
        sens = config.sensors()
        depth_topic = config.get(sens, 'depth_camera.topic')
        depth_info_topic = config.get(sens, 'depth_camera.info_topic')
        rgb_topic = config.get(sens, 'rgb_camera.topic')

        for topic in (depth_topic, depth_info_topic, rgb_topic, OBJECTS_TOPIC, MARKERS_TOPIC,
                      DETECTIONS_2D_TOPIC):
            self.assertTrue(self.probe.wait_for_publisher(topic, self.timeout(STARTUP_WALL_S)),
                            f'{topic}: 발행자 없음 (perception 스택이 뜨지 않았다)')
        gt = self.probe.subscribe('ground_truth/odom', Odometry, keep_messages=200)
        lost = self.probe.subscribe('localization/lost', Bool, 'latched')
        depth = self.probe.subscribe(depth_topic, Image, self.probe.matching_qos(depth_topic),
                                     keep_messages=12)
        info = self.probe.subscribe(depth_info_topic, CameraInfo,
                                    self.probe.matching_qos(depth_info_topic), keep_messages=2)
        objs = self.probe.subscribe(OBJECTS_TOPIC, DetectedObjectArray,
                                    self.probe.matching_qos(OBJECTS_TOPIC), keep_messages=40)
        marks = self.probe.subscribe(MARKERS_TOPIC, MarkerArray,
                                     self.probe.matching_qos(MARKERS_TOPIC), keep_messages=40)
        dets2d = self.probe.subscribe(DETECTIONS_2D_TOPIC, Detection2DArray,
                                      self.probe.matching_qos(DETECTIONS_2D_TOPIC),
                                      keep_messages=2)
        # 카메라는 주기만 본다 (raw — 역직렬화 부하로 주기가 왜곡되지 않게, 01 과 같은 방법)
        rgb = self.probe.subscribe(rgb_topic, Image, self.probe.matching_qos(rgb_topic),
                                   raw=True)

        self.wait_lifecycle_active(LIFECYCLE, 300.0)
        self.check_map_registration()
        self.wait_startup_still()
        self.warm_up_localization(stack.drive_topic, gt)   # 첫 순간 이동을 kidnap_monitor 가 보게
        self.require_topic(info, 1, STARTUP_WALL_S, depth_info_topic)
        self.require_topic(depth, 1, STARTUP_WALL_S, depth_topic)
        km = info.last()
        kin = wo.Intrinsics(km.k[0], km.k[4], km.k[2], km.k[5], km.width, km.height)
        optical = depth.last().header.frame_id
        base_to_optical = self._camera_extrinsic(optical)

        statics, actors = wo.parse_world(world_sdf.read_text(encoding='utf-8'),
                                         [str(share / 'models')])
        shapes = worldmap.footprints(world_sdf, share / 'models', config.scan_plane_height(),
                                     'both')
        views = build_viewpoints(statics, actors, shapes)
        self.measure('world_ground_truth', {
            'static_objects': len(statics), 'actors': len(actors),
            'by_class': {c: sum(1 for o in statics if o.class_name == c)
                         for c in sorted({o.class_name for o in statics})}})
        self.measure('viewpoints', [{'class': v.cls, 'target': v.target_name,
                                     'pose': [round(v.x, 2), round(v.y, 2), round(v.yaw, 3)]}
                                    for v in views])
        self.assertEqual(sorted({v.cls for v in views}), sorted(OUTPUT_CLASSES),
                         f'출력 클래스마다 시점을 만들지 못했다 (월드 지면 진실 {len(statics)} 정적 · '
                         f'{len(actors)} actor): {[v.cls for v in views]}')

        env = {'depth': depth, 'objs': objs, 'marks': marks, 'gt': gt, 'kin': kin,
               'optical': optical, 'base_to_optical': base_to_optical, 'statics': statics,
               'actors': actors, 'tf': self.probe.tf_buffer(),
               'label_params': wo.LabelParams(max_range=GT_LABEL_RANGE)}
        acc = {'rows': [], 'views': [], 'classes': set(), 'pose_frames': set(),
               'header_frames': set(), 'marker_problems': [], 'marker_frames': 0,
               'marker_object_frames': 0, 'frames': 0, 'objects': 0, 'fields_ok': False,
               'err_xy': [], 'err_by_class': {}, 'reproj': [], 'gt': {}, 'tp': {}, 'fp': {}}

        for index, view in enumerate(views):
            need = (KIDNAP_DETECT_S + RELOCALIZE_MAX_S + FACE_MAX_S + view.window_s + 60.0
                    + (ACTOR_WAIT_CAP_S if view.track is not None else 0.0))
            if not self.budget_for(need, f'view {index} ({view.cls})'):
                break
            self._teleport(world, (view.x, view.y, view.yaw), lost, f'view {index}')
            yaw_err = self._face(stack.drive_topic, gt, view.yaw)
            waited = self._wait_for_actor(view)
            summary = self._collect(index, view, env, acc)
            sample = metrics.sample_from_odom(gt.last())
            acc['views'].append([index, view.cls, view.target_name, round(sample.x, 3),
                                 round(sample.y, 3), round(sample.yaw, 4), summary['frames'],
                                 summary['objects'], summary['gt_required'],
                                 summary['matched'], '|'.join(summary['classes'])])
            self.ctx.record.write_csv('views.csv', VIEW_COLUMNS, acc['views'])
            self.ctx.record.write_csv('perception.csv', PERCEPTION_COLUMNS, acc['rows'])
            self.ctx.record.note(
                f'view {index} ({view.cls} ← {view.target_name}): 프레임 {summary["frames"]}, '
                f'객체 {summary["objects"]}, 방위 오차 {math.degrees(yaw_err):.1f}°, '
                f'작업자 대기 {"성공" if waited else "시간 초과"}')

        self._measure_rates(env, rgb, dets2d)
        self._record_measurements(acc)
        self._judge(acc)

    def _camera_extrinsic(self, optical: str):
        """base_footprint ← 광학 프레임 4×4 (정적 TF)."""
        from amr_perception import world_objects as wo
        from amr_perception.transforms import transform_from_msg
        from rclpy.time import Time as RclTime
        import tf2_ros
        buf = self.probe.tf_buffer()
        base = self.settings.frame('base_footprint')
        box = {}

        def got() -> bool:
            try:
                box['tf'] = buf.lookup_transform(base, optical, RclTime())
            except tf2_ros.TransformException:
                return False
            return True

        self.assertTrue(self.probe.wait_until(got, self.timeout(120.0), 0.2),
                        f'TF {base} ← {optical} 없음')
        c = transform_from_msg(box['tf'].transform)
        return wo.homogeneous(c.rotation, c.translation)

    def _measure_rates(self, env: dict, rgb, dets2d) -> None:
        """발행률·RTF·처리율 (측정만 — 명세 4.6 에 발행률 수치가 없고, 벽시계 발행률은 30×RTF 다)."""
        wall0, sim0 = self.probe.wall(), self.probe.now()
        ok = self.probe.sleep_ros(RATE_WINDOW_S, self.timeout(RATE_WINDOW_S * 60.0))
        wall1, sim1 = self.probe.wall(), self.probe.now()
        out: Dict[str, object] = {'sim_time_advanced': bool(ok),
                                  'rtf': round((sim1 - sim0) / max(wall1 - wall0, 1e-6), 3)}
        counts = {}
        for label, rec in (('camera', rgb), ('depth', env['depth']), ('detections_2d', dets2d),
                           ('detected_objects', env['objs']), ('markers', env['marks'])):
            st = rates.rate_stats(rates.in_window(rec.stamps(), sim0, sim1))
            wall = rates.rate_stats(rec.recv_times(wall0))
            counts[label] = wall.count
            out[label] = {'sim_hz': round(st.mean_rate, 2), 'wall_hz': round(wall.mean_rate, 2),
                          'count': wall.count}
        cam = counts.get('camera', 0)
        # 처리율 = 받은 카메라 프레임 중 검출이 나온 비율 (무차원이라 RTF 와 무관하다)
        out['processed_ratio'] = round(counts.get('detections_2d', 0) / cam, 3) if cam else None
        self.measure('publish_rates', out)

    def _record_measurements(self, acc: dict) -> None:
        """명세에 수치가 없는 값들 (판정하지 않는다)."""
        def summary(values: Sequence[float]) -> dict:
            arr = np.asarray([v for v in values if math.isfinite(v)], dtype=float)
            if arr.size == 0:
                return {'n': 0}
            return {'n': int(arr.size), 'median': cases.fmt(float(np.median(arr))),
                    'p90': cases.fmt(float(np.percentile(arr, 90))),
                    'max': cases.fmt(float(arr.max()))}

        self.measure('frames', {'processed': acc['frames'], 'objects': acc['objects'],
                                'marker_messages': acc['marker_frames']})
        self.measure('position_error_xy_m', summary(acc['err_xy']))
        self.measure('position_error_xy_m_by_class',
                     {c: summary(v) for c, v in sorted(acc['err_by_class'].items())})
        self.measure('reprojection_residual_px', summary(acc['reproj']))
        self.measure('gt_match', {c: {'gt': acc['gt'].get(c, 0), 'tp': acc['tp'].get(c, 0),
                                      'fp': acc['fp'].get(c, 0)} for c in OUTPUT_CLASSES})

    def _judge(self, acc: dict) -> None:
        """
        명세 4.6 에 수치·문장이 있는 것만 판정한다.

        판정을 모두 result.json 에 남긴 뒤 한 번에 실패시킨다 (첫 실패에서 멈추면 나머지 증거가 남지
        않는다 — 01 의 주기 판정과 같은 방식).
        """
        resolved = self.probe.node.resolve_topic_name(OBJECTS_TOPIC)
        types = sorted({i.topic_type
                        for i in self.probe.node.get_publishers_info_by_topic(resolved)})
        classes = sorted(acc['classes'] & set(OUTPUT_CLASSES))
        pose_frames = sorted(acc['pose_frames'])
        problems = sorted(set(acc['marker_problems']))
        # 하네스 선택(명세 아님): 명세 4.6 에 3D 위치 오차 수치가 없다 — 변환 배관 확인용 느슨한 가드
        errs = [v for v in acc['err_xy'] if math.isfinite(v)]
        median = float(np.median(errs)) if errs else math.nan
        checks = [
            ('custom message type (명세 4.6 "커스텀 메시지 타입으로 발행")',
             {'type': types, 'fields_filled': acc['fields_ok']},
             {'type': [CUSTOM_MSG], 'fields_filled': True},
             types == [CUSTOM_MSG] and acc['fields_ok'], ''),
            ('detected classes (명세 4.6 "화물, 사람, 표지판 등 3종 이상")', classes,
             f'>= {MIN_CLASSES} 종 of {list(OUTPUT_CLASSES)}', len(classes) >= MIN_CLASSES, ''),
            ('pose_3d frame (명세 4.6 "3D 공간 좌표(Map frame)")', pose_frames, [MAP_FRAME],
             pose_frames == [MAP_FRAME], ''),
            ('markers: 객체마다 map 프레임 CUBE + TEXT (명세 4.6 "지도상에 마커로 표시")',
             problems or 'ok', [], not problems and acc['marker_object_frames'] > 0, ''),
            ('[하네스 선택, 명세 아님] 3D xy 오차 중앙값 (GT 매칭, IoU 0.5)', cases.fmt(median),
             f'<= {GT_ERR_GUARD_M} (배관 확인용, 명세 4.6 에는 수치 없음)',
             bool(errs) and median <= GT_ERR_GUARD_M, 'm'),
        ]
        failed = []
        for name, value, threshold, passed, unit in checks:
            self.ctx.record.check(name, cases.fmt(value), cases.fmt(threshold), passed, unit)
            if not passed:
                failed.append(f'{name}: {cases.fmt(value)} (기준 {cases.fmt(threshold)})')
        self.assertFalse(
            failed,
            f'{failed} — 프레임 {acc["frames"]} · 객체 {acc["objects"]} · 마커 메시지 '
            f'{acc["marker_frames"]} (객체 있는 것 {acc["marker_object_frames"]}) · '
            f'GT 매칭 {len(errs)} · detected_objects 헤더 프레임 {sorted(acc["header_frames"])}')

    # ------------------------------------------------------------ 추론 속도

    def _camera_frames(self, n: int) -> list:
        """카메라 영상 n 장 (추론 벤치 입력 — 주기 측정용 raw 구독과 별개의 임시 구독)."""
        from amr_itest.probe import qos as probe_qos
        topic = config.get(config.sensors(), 'rgb_camera.topic')
        frames: list = []
        sub = self.probe.node.create_subscription(
            Image, topic, frames.append, probe_qos(self.probe.matching_qos(topic)))
        try:
            self.probe.wait_until(lambda: len(frames) >= n, self.timeout(240.0), 0.1)
        finally:
            self.probe.node.destroy_subscription(sub)
        return list(frames)[:n]

    def test_20_inference_speed(self) -> None:
        """
        추론 속도 (명세 4.6 "CPU 기준 10 FPS 이상(또는 GPU 가속 시 30 FPS 이상)").

        배포 설정(perception.yaml /**/yolo_node) 그대로 만든 YoloDetector 로 실제 Gazebo 카메라 프레임을
        추론해 프레임별 벽시계 지연을 재고 중앙값의 역수를 FPS 로 본다. 토픽 발행률로 재면 카메라가 sim
        30 Hz = 벽시계 30×RTF Hz 라 RTF 를 FPS 로 보고하게 된다 (perception.md §8.2: RTF 0.23~0.33 에서
        카메라 1.4~7.1 Hz) — 그래서 추론 시간을 직접 잰다. 돌고 있는 yolo_node 와 장치를 나눠 쓰므로 이
        값은 단독 벤치(§8.2 GPU 1.9 ms / CPU 320 9.1 ms)보다 보수적이다.
        """
        from amr_perception.class_mapping import ClassMapper
        from amr_perception.yolo_backend import resolve_weights, YoloDetector
        from cv_bridge import CvBridge
        share = req.share_dir('amr_perception')
        params = config.load_ros_params(req.config_file('amr_perception', 'perception.yaml'),
                                        '/**/yolo_node')
        mapper = ClassMapper.from_yaml(str(share / 'config' / 'classes.yaml'))
        search = [str(share / 'models'), str(config.repo_root() / 'models')]
        weights = resolve_weights(str(params['weights']), search)
        if not os.path.exists(weights):
            weights = resolve_weights(str(params['fallback_weights']), search)
        messages = self._camera_frames(INFER_FRAMES)
        self.assertGreaterEqual(len(messages), 10, f'카메라 프레임 부족 ({len(messages)})')
        bridge = CvBridge()
        images = [bridge.imgmsg_to_cv2(m, desired_encoding='bgr8') for m in messages]
        detector = YoloDetector(
            weights, str(params['device']), int(params['imgsz']), int(params['cpu_imgsz']),
            float(params['conf_thresh']), float(params['iou']), bool(params['half']),
            int(params['max_det']), mapper, int(params['torch_threads']),
            int(params['cpu_threads']), bool(params['fast_path']))
        detector.warmup()          # CUDA 초기화·첫 할당 비용은 재지 않는다
        times = []
        for image in images:
            t0 = time.perf_counter()
            detector.infer(image)
            times.append((time.perf_counter() - t0) * 1e3)
        self.ctx.record.write_csv('inference.csv', ['i', 'infer_ms'],
                                  [[i, round(v, 3)] for i, v in enumerate(times)])
        median = float(np.median(times))
        fps = 1000.0 / median if median > 0.0 else math.inf
        limit = GPU_FPS if detector.on_gpu else CPU_FPS
        path = 'GPU' if detector.on_gpu else 'CPU'
        self.measure('inference', {
            'device': detector.device, 'path': path, 'imgsz': detector.imgsz,
            'half': detector.half, 'weights': os.path.basename(weights), 'frames': len(times),
            'median_ms': cases.fmt(median, 2),
            'p95_ms': cases.fmt(float(np.percentile(times, 95)), 2),
            'max_ms': cases.fmt(max(times), 2), 'fps': cases.fmt(fps, 2)})
        self.check(f'YOLO inference FPS ({path} 경로, 명세 4.6)', fps, f'>= {limit}',
                   fps >= limit, 'FPS',
                   f'프레임 {len(times)} 장 추론 중앙값 {median:.1f} ms (벽시계, RTF 무관)')


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
