"""
검출 객체 시각화 규칙 (RViz MarkerArray 의 순수 데이터 부분, ROS 비의존).

명세 8장 예시: 'Box' 가 감지되면 지도 위 해당 위치에 빨간 육면체 + 텍스트
"Class: Box, Conf: 0.92, Dist: 1.5m". 클래스별 색·크기는 여기 한 곳에서 정한다.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

# 클래스별 (R, G, B, A) 와 기본 크기 (x, y, z) [m] — 명세 8장 물품 표(중형 박스)·사람·표지판 판
CLASS_STYLE: Dict[str, Tuple[Tuple[float, float, float, float], Tuple[float, float, float]]] = {
    'box': ((0.9, 0.1, 0.1, 0.8), (0.5, 0.4, 0.3)),
    'person': ((0.1, 0.8, 0.2, 0.8), (0.5, 0.5, 1.7)),
    'sign': ((0.1, 0.3, 0.9, 0.8), (0.05, 0.6, 0.6)),
}
DEFAULT_STYLE = ((0.8, 0.8, 0.1, 0.8), (0.3, 0.3, 0.3))


def format_label(class_name: str, confidence: float, distance: float,
                 capitalize: bool = False) -> str:
    """
    마커 텍스트 "Class: box, Conf: 0.92, Dist: 1.5m".

    capitalize=True 이면 명세 예시처럼 첫 글자를 대문자로 ("Class: Box, ...").
    """
    name = class_name[:1].upper() + class_name[1:] if capitalize else class_name
    return f'Class: {name}, Conf: {confidence:.2f}, Dist: {distance:.1f}m'


@dataclass
class MarkerSpec:
    """CUBE + TEXT 마커 한 쌍의 기하·색."""

    color: Tuple[float, float, float, float]
    scale: Tuple[float, float, float]
    cube_z: float       # 육면체 중심 z (바닥에 놓이도록)
    text_z: float       # 텍스트 z (육면체 위)
    text: str


def marker_spec(class_name: str, confidence: float, distance: float, center_z: float,
                ground_z: float = 0.0, text_height: float = 0.25,
                capitalize: bool = False) -> MarkerSpec:
    """
    클래스 스타일로 마커 명세를 만든다.

    육면체는 바닥(ground_z)에 앉히되 추정 중심 z 가 더 높으면 그 높이를 따른다 (선반 위 박스).
    """
    color, scale = CLASS_STYLE.get(class_name, DEFAULT_STYLE)
    cube_z = max(center_z, ground_z + 0.5 * scale[2])
    text_z = cube_z + 0.5 * scale[2] + 0.6 * text_height
    return MarkerSpec(color, scale, cube_z, text_z,
                      format_label(class_name, confidence, distance, capitalize))
