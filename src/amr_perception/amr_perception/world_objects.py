"""
Gazebo 월드 SDF → 라벨 대상 3D 박스 목록 (Gazebo 지면 진실 데이터셋 생성용, ROS 비의존).

- <include> 모델: uri(model://<name>) 가 class_rules 의 접두사와 맞으면 그 클래스. 크기는 모델 SDF 의 첫
  <box><size> 에서, 박스 중심 높이는 모델 링크/비주얼 pose 의 z 에서 읽는다 (창고 박스 모델은 원점 = 바닥).
- <actor>: <trajectory> 웨이포인트를 시각으로 선형 보간 (loop 이면 마지막 웨이포인트 시각으로 나눈 나머지).
  작업자(worker) 는 person, 크기는 person_size.
월드 좌표 = Gazebo world (ground_truth/odom 의 frame_id 'world').
"""

from __future__ import annotations

from dataclasses import dataclass
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
PERSON_SIZE = (0.5, 0.5, 1.75)


def _floats(text: Optional[str], n: int = 6) -> List[float]:
    vals = [float(v) for v in (text or '').split()]
    return (vals + [0.0] * n)[:n]


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


def classify(uri_name: str, rules: Dict[str, str]) -> Optional[str]:
    """모델 이름 → 클래스 (가장 긴 접두사 규칙)."""
    best = None
    for prefix, cls in rules.items():
        if uri_name.startswith(prefix) and (best is None or len(prefix) > len(best[0])):
            best = (prefix, cls)
    return None if best is None else best[1]


def parse_world(sdf_text: str, model_dirs: Sequence[str] = (),
                class_rules: Optional[Dict[str, str]] = None
                ) -> Tuple[List[WorldObject], List[ActorTrack]]:
    """월드 SDF 문자열 → (정적 라벨 물체, 작업자 궤적)."""
    rules = class_rules or DEFAULT_CLASS_RULES
    root = ET.fromstring(sdf_text)
    world = root.find('world') if root.tag == 'sdf' else root
    objects: List[WorldObject] = []
    cache: Dict[str, Optional[Tuple[np.ndarray, float]]] = {}
    for inc in world.findall('include'):
        uri = inc.findtext('uri') or ''
        m = re.match(r'model://([^/]+)', uri.strip())
        if m is None:
            continue
        mname = m.group(1)
        cls = classify(mname, rules)
        if cls is None:
            continue
        if mname not in cache:
            cache[mname] = None
            for d in model_dirs:
                path = os.path.join(d, mname, 'model.sdf')
                if os.path.exists(path):
                    with open(path, 'r', encoding='utf-8') as f:
                        cache[mname] = model_box(f.read())
                    break
        box = cache[mname]
        if box is None:
            continue
        size, zoff = box
        x, y, z, _, _, yaw = _floats(inc.findtext('pose'))
        objects.append(WorldObject(cls, np.array([x, y, z + zoff]), size, yaw,
                                   inc.findtext('name') or mname))
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


def actor_objects(actors: Sequence[ActorTrack], t: float,
                  person_size: Tuple[float, float, float] = PERSON_SIZE) -> List[WorldObject]:
    """시각 t 의 작업자 → person 3D 박스 (발 위치 + 높이/2)."""
    out = []
    for a in actors:
        p, yaw = a.pose_at(t)
        out.append(WorldObject('person', p + np.array([0.0, 0.0, 0.5 * person_size[2]]),
                               np.array(person_size), yaw, a.name))
    return out
