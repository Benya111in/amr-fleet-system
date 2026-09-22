"""
Gazebo 월드 SDF → 라벨 대상 3D 박스, 깊이 이미지로 검증한 2D 지면 진실 라벨 (YOLO 데이터셋·실시간 재현율 평가, ROS 비의존).

파싱 (parse_world)
- <include> 모델: uri(model://<name>) 가 class_rules 의 접두사와 맞으면 그 클래스 (box_small/medium/large → box).
  크기·중심은 모델 SDF 시각체 외곽(AABB, box/cylinder/sphere, 링크·시각체 pose 반영)에서 읽는다.
- 인라인 <model>: 이름이 inline_rules 접두사(sign_)와 맞으면 box 시각체(판) 하나하나가 그 클래스 물체다.
  창고 월드의 표지판 25 개는 전부 인라인 판이라 include 만 읽으면 sign 라벨이 0 이 된다 (리뷰 지적).
- dynamic_models: 물리로 움직이는 include(지게차·셔틀 AMR)는 크기·원점 오프셋만 템플릿(ModelTemplate)으로 두고
  자세는 실행 중 odom 으로 받는다 (template.at(x, y, yaw)).
- <actor>: <trajectory> 웨이포인트를 시각으로 선형 보간 (loop 이면 마지막 웨이포인트 시각으로 나눈 나머지). 월드
  생성기 웨이포인트 간격이 0.5 s 라 Gazebo 스플라인(Catmull-Rom)과의 차는 곡선 구간에서도 1 cm 미만이다.
월드 좌표 = Gazebo world = map (C4: map 프레임 == 월드 프레임).

라벨 (label_objects) — 투영 + 깊이 검증
  물체 3D 박스와 픽셀 광선의 교차 구간 [t_in, t_out] (광학 Z, slab 방법) 을 측정 깊이 d 와 비교한다.
    물체 픽셀 : d 의 3D 점이 여유 tol 만큼 부풀린 박스 안 (받침면 층 floor_margin 은 뺀다 — 바닥·선반 판)
    가림 픽셀 : d < t_in − tol (박스 앞에 다른 물체)
    통과 픽셀 : 그 외 (박스 안 빈 공간: 사람 팔다리 사이, 지게차 포크 사이, 깊이 무효)
  라벨 bbox = 물체 픽셀 외접 사각형 (보이는 부분에 꼭 맞는 상자, 이미지 안으로 잘림).
  가시율 = 물체 / (물체 + 가림), 잘림 = 1 − (이미지 안 투영 면적 / 전체 투영 면적).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
import re
from typing import Dict, List, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

import numpy as np

from .synthetic import WorldObject

DEFAULT_CLASS_RULES: Dict[str, str] = {
    'box_small': 'box', 'box_medium': 'box', 'box_large': 'box', 'box': 'box', 'sign': 'sign',
    'stop_sign': 'sign',
}
# 인라인 <model name=...> 접두사 → 클래스 (판 시각체마다 물체 하나)
DEFAULT_INLINE_RULES: Dict[str, str] = {'sign_': 'sign'}
PERSON_SIZE = (0.5, 0.5, 1.75)
# 데이터셋 클래스 (순서 = 모델 class id). 출력 3 종(classes.yaml)은 앞 3 개이고, forklift·amr 은 상자와의 혼동을
# 줄이려고 따로 학습한다 (classes.yaml 에 없으면 yolo_node 가 버린다).
DATASET_CLASSES = ['box', 'person', 'sign', 'forklift', 'amr']


def _floats(text: Optional[str], n: int = 6) -> List[float]:
    vals = [float(v) for v in (text or '').split()]
    return (vals + [0.0] * n)[:n]


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """SDF rpy (고정축 X-Y-Z) → 회전 행렬 R = Rz·Ry·Rx."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _pose_tf(text: Optional[str]) -> Tuple[np.ndarray, np.ndarray]:
    p = _floats(text)
    return rpy_matrix(p[3], p[4], p[5]), np.array(p[:3])


@dataclass
class ActorTrack:
    """actor 궤적 (웨이포인트 시각·위치·yaw)."""

    name: str
    times: np.ndarray
    xyz: np.ndarray
    yaw: np.ndarray
    loop: bool = True
    delay: float = 0.0

    def pose_at(self, t: float) -> Tuple[np.ndarray, float]:
        """Sim 시각 t 의 (위치, yaw) — 구간 선형 보간."""
        tt = max(t - self.delay, 0.0)
        duration = float(self.times[-1])
        if self.loop and duration > 0.0:
            tt = math.fmod(tt, duration)
        tt = min(tt, duration)
        i = int(np.searchsorted(self.times, tt, side='right'))
        if i <= 0:
            return self.xyz[0].copy(), float(self.yaw[0])
        if i >= len(self.times):
            return self.xyz[-1].copy(), float(self.yaw[-1])
        t0, t1 = self.times[i - 1], self.times[i]
        a = 0.0 if t1 <= t0 else (tt - t0) / (t1 - t0)
        dyaw = math.atan2(math.sin(self.yaw[i] - self.yaw[i - 1]),
                          math.cos(self.yaw[i] - self.yaw[i - 1]))
        return (1 - a) * self.xyz[i - 1] + a * self.xyz[i], float(self.yaw[i - 1] + a * dyaw)


@dataclass
class ModelTemplate:
    """움직이는 모델의 라벨 박스 (모델 원점 기준 중심 오프셋·크기). 자세는 실행 중 주어진다."""

    class_name: str
    offset: np.ndarray          # 모델 원점 → 박스 중심 (모델 좌표)
    size: np.ndarray
    name: str = ''

    def at(self, x: float, y: float, yaw: float, z: float = 0.0) -> WorldObject:
        """모델 원점 자세 (x, y, z, yaw) → 월드 3D 박스."""
        c, s = math.cos(yaw), math.sin(yaw)
        ox, oy, oz = self.offset
        return WorldObject(self.class_name, np.array([x + c * ox - s * oy, y + s * ox + c * oy,
                                                      z + oz]), self.size.copy(), yaw, self.name)


def model_box(model_sdf: str) -> Optional[Tuple[np.ndarray, float]]:
    """모델 SDF 문자열 → (박스 크기, 박스 중심 z 오프셋). 첫 <box><size> 와 그 요소의 pose z 합."""
    root = ET.fromstring(model_sdf)
    for parent in root.iter():
        for geom in parent.findall('geometry'):
            size = geom.find('box/size')
            if size is None:
                continue
            z = _floats(parent.findtext('pose'))[2]
            link = next((lk for lk in root.iter('link') if parent in list(lk.iter())), None)
            if link is not None and link is not parent:
                z += _floats(link.findtext('pose'))[2]
            return np.array(_floats(size.text, 3)), z
    return None


def _geometry_half_extent(geom: ET.Element) -> Optional[np.ndarray]:
    size = geom.findtext('box/size')
    if size is not None:
        return 0.5 * np.array(_floats(size, 3))
    if geom.find('cylinder') is not None:
        r = float(geom.findtext('cylinder/radius') or 0.0)
        return np.array([r, r, 0.5 * float(geom.findtext('cylinder/length') or 0.0)])
    if geom.find('sphere') is not None:
        r = float(geom.findtext('sphere/radius') or 0.0)
        return np.array([r, r, r])
    return None


def model_extent(model_sdf: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    모델 SDF → 모델 좌표 AABB (중심, 크기). 모든 링크의 box/cylinder/sphere 시각체를 링크·시각체 pose 로 옮겨 합친다.

    메시 시각체는 건너뛴다. 시각체가 하나도 없으면 None.
    """
    root = ET.fromstring(model_sdf)
    model = root.find('model') if root.tag == 'sdf' else root
    if model is None:
        return None
    pts = []
    for link in model.iter('link'):
        r_l, t_l = _pose_tf(link.findtext('pose'))
        for vis in link.findall('visual'):
            geom = vis.find('geometry')
            half = None if geom is None else _geometry_half_extent(geom)
            if half is None:
                continue
            r_v, t_v = _pose_tf(vis.findtext('pose'))
            corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                                for sz in (-1, 1)]) * half
            pts.append((corners @ r_v.T + t_v) @ r_l.T + t_l)
    if not pts:
        return None
    allp = np.vstack(pts)
    lo, hi = allp.min(axis=0), allp.max(axis=0)
    return 0.5 * (lo + hi), hi - lo


def classify(uri_name: str, rules: Dict[str, str]) -> Optional[str]:
    """모델 이름 → 클래스 (가장 긴 접두사 규칙)."""
    best = None
    for prefix, cls in rules.items():
        if uri_name.startswith(prefix) and (best is None or len(prefix) > len(best[0])):
            best = (prefix, cls)
    return None if best is None else best[1]


def _read_model(mname: str, model_dirs: Sequence[str]) -> Optional[str]:
    for d in model_dirs:
        path = os.path.join(d, mname, 'model.sdf')
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                return f.read()
    return None


def _world_element(sdf_text: str) -> ET.Element:
    root = ET.fromstring(sdf_text)
    return root.find('world') if root.tag == 'sdf' else root


def _include_uri_name(inc: ET.Element) -> Optional[str]:
    m = re.match(r'model://([^/]+)', (inc.findtext('uri') or '').strip())
    return None if m is None else m.group(1)


def inline_objects(world: ET.Element, rules: Dict[str, str]) -> List[WorldObject]:
    """인라인 <model> 중 규칙에 맞는 것의 box 시각체 → 3D 박스 (판 표지판). 월드 자세는 yaw 만 있는 판을 가정."""
    out: List[WorldObject] = []
    for model in world.findall('model'):
        name = model.get('name', '')
        cls = classify(name, rules)
        if cls is None:
            continue
        r_m, t_m = _pose_tf(model.findtext('pose'))
        for link in model.findall('link'):
            r_l, t_l = _pose_tf(link.findtext('pose'))
            for vis in link.findall('visual'):
                size = vis.findtext('geometry/box/size')
                if size is None:
                    continue
                r_v, t_v = _pose_tf(vis.findtext('pose'))
                rot = r_m @ r_l @ r_v
                center = r_m @ (r_l @ t_v + t_l) + t_m
                yaw = math.atan2(rot[1, 0], rot[0, 0])
                out.append(WorldObject(cls, center, np.array(_floats(size, 3)), yaw,
                                       f'{name}/{vis.get("name", "visual")}'))
    return out


def parse_world(sdf_text: str, model_dirs: Sequence[str] = (),
                class_rules: Optional[Dict[str, str]] = None,
                inline_rules: Optional[Dict[str, str]] = None
                ) -> Tuple[List[WorldObject], List[ActorTrack]]:
    """월드 SDF 문자열 → (정적 라벨 물체, 작업자 궤적). 정적 = include 모델 + 인라인 판."""
    rules = class_rules or DEFAULT_CLASS_RULES
    world = _world_element(sdf_text)
    objects: List[WorldObject] = []
    cache: Dict[str, Optional[Tuple[np.ndarray, float]]] = {}
    for inc in world.findall('include'):
        mname = _include_uri_name(inc)
        if mname is None:
            continue
        cls = classify(mname, rules)
        if cls is None:
            continue
        if mname not in cache:
            text = _read_model(mname, model_dirs)
            cache[mname] = None if text is None else model_box(text)
        box = cache[mname]
        if box is None:
            continue
        size, zoff = box
        x, y, z, _, _, yaw = _floats(inc.findtext('pose'))
        objects.append(WorldObject(cls, np.array([x, y, z + zoff]), size, yaw,
                                   inc.findtext('name') or mname))
    objects += inline_objects(world, DEFAULT_INLINE_RULES if inline_rules is None
                              else inline_rules)
    actors: List[ActorTrack] = []
    for act in world.findall('actor'):
        traj = act.find('script/trajectory')
        if traj is None:
            continue
        wps = [(float(w.findtext('time')), _floats(w.findtext('pose')))
               for w in traj.findall('waypoint')]
        if not wps:
            continue
        wps.sort(key=lambda w: w[0])
        loop = (act.findtext('script/loop') or 'true').strip().lower() == 'true'
        delay = float(act.findtext('script/delay_start') or 0.0)
        actors.append(ActorTrack(
            act.get('name', 'actor'), np.array([w[0] for w in wps]),
            np.array([w[1][:3] for w in wps]), np.array([w[1][5] for w in wps]), loop, delay))
    return objects, actors


def dynamic_templates(sdf_text: str, model_dirs: Sequence[str],
                      models: Dict[str, str]) -> Dict[str, ModelTemplate]:
    """움직이는 include 이름 → 클래스 (예 {'forklift_main': 'forklift'}) 의 라벨 박스 템플릿 (시각체 AABB)."""
    world = _world_element(sdf_text)
    out: Dict[str, ModelTemplate] = {}
    for inc in world.findall('include'):
        name = inc.findtext('name') or ''
        mname = _include_uri_name(inc)
        if name not in models or mname is None:
            continue
        text = _read_model(mname, model_dirs)
        ext = None if text is None else model_extent(text)
        if ext is not None:
            out[name] = ModelTemplate(models[name], ext[0], ext[1], name)
    return out


def actor_objects(actors: Sequence[ActorTrack], t: float,
                  person_size: Tuple[float, float, float] = PERSON_SIZE) -> List[WorldObject]:
    """시각 t 의 작업자 → person 3D 박스 (발 위치 + 높이/2)."""
    out = []
    for a in actors:
        p, yaw = a.pose_at(t)
        out.append(WorldObject('person', p + np.array([0.0, 0.0, 0.5 * person_size[2]]),
                               np.array(person_size), yaw, a.name))
    return out


# ------------------------------------------------------------------ 깊이 검증 라벨

@dataclass
class Intrinsics:
    """핀홀 내부 파라미터 (camera_info K) 와 이미지 크기."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass
class LabelParams:
    """깊이 검증 라벨 규칙 (scripts/dataset_capture.py 기본값과 근거)."""

    tol: float = 0.04               # [m] 박스 여유: 깊이 노이즈 σ 0.005 + 모델 치수·자세 오차
    tol_quad: float = 0.001         # [1/m] 거리 제곱 여유 (먼 물체 깊이 양자화·보간)
    floor_margin: float = 0.03      # [m] 받침면(바닥·선반 판) 층 — 박스 밑면에서 이 두께 안의 점은 물체가 아니다
    min_visible: float = 0.2        # 가시율 하한 (보이는 부분이 이보다 적으면 라벨 없음)
    max_truncation: float = 0.75    # 잘림 상한 (이미지 안에 투영의 25 % 미만이면 라벨 없음)
    min_pixels: int = 20            # 물체 픽셀 수 하한 (stride 1 환산)
    min_side_px: float = 6.0        # [px] bbox 짧은 변 하한
    big_side_px: float = 48.0       # [px] 보이는 bbox 짧은 변이 이 이상이면 가림·잘림 규칙을 건너뛴다
    max_range: float = 60.0         # [m] 카메라 → 박스 중심 거리 상한
    near: float = 0.05              # [m] 광선 시작 (카메라 앞)
    stride_px: int = 2              # 큰 사각형(> 96 px)에서 픽셀 표본 간격


@dataclass
class GtLabel:
    """한 물체의 지면 진실 라벨 판정."""

    class_name: str
    name: str
    bbox: Tuple[float, float, float, float]      # 보이는 부분 [x1, y1, x2, y2] (px, 이미지 안)
    full_bbox: Tuple[float, float, float, float]  # 3D 박스 투영 (잘리기 전)
    visible: float
    truncated: float
    distance: float                               # [m] 카메라 → 박스 중심
    pixels: int
    kept: bool
    reason: str = ''
    extra: Dict[str, float] = field(default_factory=dict)


def _object_frame(obj: WorldObject) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    c, s = math.cos(obj.yaw), math.sin(obj.yaw)
    r_wo = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return r_wo, np.asarray(obj.center, dtype=float), 0.5 * np.asarray(obj.size, dtype=float)


def project_extent(obj: WorldObject, r_cw: np.ndarray, t_cw: np.ndarray, k: Intrinsics,
                   near: float = 0.05) -> Optional[Tuple[float, float, float, float]]:
    """3D 박스 8 꼭짓점 투영 bbox (자르기 전). 카메라 앞 꼭짓점이 없으면 None, 일부가 뒤면 무한대 방향으로 편다."""
    r_wo, ctr, half = _object_frame(obj)
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                        for sz in (-1, 1)]) * half @ r_wo.T + ctr
    pc = corners @ r_cw.T + t_cw
    front = pc[:, 2] > near
    if not np.any(front):
        return None
    if not np.all(front):
        return (-np.inf, -np.inf, np.inf, np.inf)
    u = k.fx * pc[:, 0] / pc[:, 2] + k.cx
    v = k.fy * pc[:, 1] / pc[:, 2] + k.cy
    return float(u.min()), float(v.min()), float(u.max()), float(v.max())


def _truncation(full: Tuple[float, float, float, float], k: Intrinsics) -> float:
    x1, y1, x2, y2 = full
    if not all(math.isfinite(v) for v in full):
        return 1.0
    area = (x2 - x1) * (y2 - y1)
    if area <= 0.0:
        return 1.0
    cx1, cy1 = max(x1, 0.0), max(y1, 0.0)
    cx2, cy2 = min(x2, float(k.width)), min(y2, float(k.height))
    inside = max(cx2 - cx1, 0.0) * max(cy2 - cy1, 0.0)
    return 1.0 - inside / area


def label_object(obj: WorldObject, r_cw: np.ndarray, t_cw: np.ndarray, k: Intrinsics,
                 depth: np.ndarray, p: LabelParams = LabelParams(),
                 supported: bool = True) -> Optional[GtLabel]:
    """
    물체 하나 → 깊이 검증 라벨. 투영이 이미지와 겹치지 않으면 None.

    r_cw, t_cw: 월드 → 광학 프레임 (X_c = r_cw·X_w + t_cw). depth: 광학 Z [m] (무효 = inf/nan/0).
    supported: 받침면 위에 놓인 물체 (박스·사람·차량). 판 표지판은 False (밑면 층을 빼지 않는다).
    """
    r_wo, ctr, half = _object_frame(obj)
    cam_w = -r_cw.T @ t_cw
    dist = float(np.linalg.norm(ctr - cam_w))
    if dist > p.max_range:
        return None
    full = project_extent(obj, r_cw, t_cw, k, p.near)
    if full is None:
        return None
    x1 = int(math.floor(max(full[0], 0.0)))
    y1 = int(math.floor(max(full[1], 0.0)))
    x2 = int(math.ceil(min(full[2], float(k.width))))
    y2 = int(math.ceil(min(full[3], float(k.height))))
    if x2 <= x1 or y2 <= y1:
        return None
    stride = p.stride_px if max(x2 - x1, y2 - y1) > 96 else 1
    us = np.arange(x1, x2, stride) + 0.5 * stride
    vs = np.arange(y1, y2, stride) + 0.5 * stride
    uu, vv = np.meshgrid(us, vs)
    # 광선 (광학 프레임, z = 1 이라 광선 매개변수 t = 광학 Z)
    d_c = np.stack([(uu - k.cx) / k.fx, (vv - k.cy) / k.fy, np.ones_like(uu)], axis=-1)
    r_ow = r_wo.T
    d_o = d_c @ (r_ow @ r_cw.T).T                       # 물체 좌표 광선 방향
    o_o = r_ow @ (cam_w - ctr)                          # 물체 좌표 카메라 중심
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / d_o
        t1 = (-half - o_o) * inv
        t2 = (half - o_o) * inv
    t_in = np.nanmax(np.minimum(t1, t2), axis=-1)
    t_out = np.nanmin(np.maximum(t1, t2), axis=-1)
    hit = (t_out >= np.maximum(t_in, p.near))
    di = np.clip(np.floor(vv).astype(int), 0, k.height - 1)
    dj = np.clip(np.floor(uu).astype(int), 0, k.width - 1)
    d = np.asarray(depth, dtype=np.float64)[di, dj]
    valid = np.isfinite(d) & (d > 0.0)
    d = np.where(valid, d, np.inf)
    tol = p.tol + p.tol_quad * dist * dist
    pt = o_o + np.where(valid, d, 0.0)[..., None] * d_o
    lo = -half - tol
    if supported:
        lo = lo.copy()
        lo[2] = -half[2] + p.floor_margin
    inside = valid & np.all((pt >= lo) & (pt <= half + tol), axis=-1)
    obj_px = hit & inside
    occ_px = hit & ~inside & (d < t_in - tol)
    scale = stride * stride
    n_obj, n_occ = int(np.count_nonzero(obj_px)) * scale, int(np.count_nonzero(occ_px)) * scale
    visible = n_obj / max(n_obj + n_occ, 1)
    truncated = _truncation(full, k)
    if n_obj == 0:
        bbox = (0.0, 0.0, 0.0, 0.0)
    else:
        su, sv = uu[obj_px], vv[obj_px]
        hs = 0.5 * stride
        bbox = (max(float(su.min()) - hs, 0.0), max(float(sv.min()) - hs, 0.0),
                min(float(su.max()) + hs, float(k.width)), min(float(sv.max()) + hs,
                                                               float(k.height)))
    reason = ''
    side = min(bbox[2] - bbox[0], bbox[3] - bbox[1])
    big = side >= p.big_side_px          # 보이는 부분이 크면(가까운 사람 다리 등) 가림·잘림 비율과 무관하게 라벨
    if n_obj < p.min_pixels:
        reason = 'pixels'
    elif side < p.min_side_px:
        reason = 'small'
    elif visible < p.min_visible and not big:
        reason = 'occluded'
    elif truncated > p.max_truncation and not big:
        reason = 'truncated'
    return GtLabel(obj.class_name, obj.name, bbox, full, visible, truncated, dist, n_obj,
                   reason == '', reason)


def label_objects(objects: Sequence[WorldObject], r_cw: np.ndarray, t_cw: np.ndarray,
                  k: Intrinsics, depth: np.ndarray, p: LabelParams = LabelParams(),
                  unsupported_classes: Sequence[str] = ('sign',)) -> List[GtLabel]:
    """여러 물체 → 이미지와 겹치는 물체의 라벨 판정 목록 (kept 로 채택 여부)."""
    out = []
    for obj in objects:
        lab = label_object(obj, r_cw, t_cw, k, depth, p,
                           supported=obj.class_name not in unsupported_classes)
        if lab is not None:
            out.append(lab)
    return out


def camera_from_base(x: float, y: float, yaw: float, base_to_optical: np.ndarray,
                     z: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
    """
    로봇 base_footprint 월드 자세 (x, y, z, yaw) + 4×4 base_footprint→광학 변환 → (r_cw, t_cw) 월드→광학.

    base_to_optical 은 광학 프레임 점을 base_footprint 로 옮기는 변환 (TF lookup(base, optical) 과 같은 방향).
    """
    c, s = math.cos(yaw), math.sin(yaw)
    t_wb = np.eye(4)
    t_wb[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    t_wb[:3, 3] = [x, y, z]
    return world_to_camera(t_wb, base_to_optical)


def world_to_camera(t_wb: np.ndarray, base_to_optical: np.ndarray
                    ) -> Tuple[np.ndarray, np.ndarray]:
    """4×4 월드←base (지면 진실 전체 자세) ∘ base←광학 → (r_cw, t_cw) 월드→광학."""
    t_wc = np.asarray(t_wb, dtype=float) @ np.asarray(base_to_optical, dtype=float)
    r_cw = t_wc[:3, :3].T
    return r_cw, -r_cw @ t_wc[:3, 3]


def homogeneous(rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
    """회전 + 이동 → 4×4."""
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return out


def iou(a: Sequence[float], b: Sequence[float]) -> float:
    """두 [x1, y1, x2, y2] 상자의 IoU."""
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0.0 else 0.0


def match_detections(gts: Sequence[Tuple[str, Sequence[float], bool]],
                     dets: Sequence[Tuple[str, Sequence[float], float]],
                     iou_thresh: float = 0.5) -> Tuple[List[int], List[int]]:
    """
    같은 클래스끼리 점수 내림차순 탐욕 매칭 (COCO 평가 규칙).

    gts: (클래스, bbox, 필수 여부) — 필수가 아닌 GT(작음·많이 가림·잘림)는 맞히면 무시(TP/FP 둘 다 아님).
    dets: (클래스, bbox, 점수).
    반환: (GT 마다 매칭된 det 번호 또는 −1, det 마다 매칭된 GT 번호 또는 −1).
    """
    gt_match = [-1] * len(gts)
    det_match = [-1] * len(dets)
    for di in sorted(range(len(dets)), key=lambda i: -dets[i][2]):
        dcls, dbox = dets[di][0], dets[di][1]
        # 1) 아직 안 맞은 필수 GT 중 IoU 최대, 2) 없으면 무시 GT (여러 검출이 같은 무시 GT 에 붙어도 된다)
        for want_required in (True, False):
            cands = [(iou(g[1], dbox), gi) for gi, g in enumerate(gts)
                     if g[0] == dcls and g[2] == want_required
                     and not (want_required and gt_match[gi] >= 0)]
            cands = [c for c in cands if c[0] >= iou_thresh]
            if cands:
                gi = max(cands)[1]
                det_match[di] = gi
                if want_required:
                    gt_match[gi] = di
                break
    return gt_match, det_match
