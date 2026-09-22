"""
검출 모델 클래스 → 명세 클래스 매핑 (config/classes.yaml, ROS 비의존).

모델(COCO 사전학습 yolov8n.pt 또는 합성 데이터로 미세조정한 가중치)의 클래스 이름을 출력 클래스
(box / person / sign, 순서 = 출력 class_id) 로 옮긴다. 매핑에 없는 모델 클래스는 버린다.
이름 기준이므로 같은 파일이 COCO 가중치(person, stop sign …)와 미세조정 가중치(box, person, sign)
양쪽에 그대로 맞는다. COCO id 는 classes.yaml 의 coco_ids 로 문서화하고 테스트가 이름과 대조한다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Tuple

import yaml


@dataclass
class ClassMapper:
    """모델 클래스 id/이름 → (출력 id, 출력 이름)."""

    classes: List[str]
    name_map: Dict[str, str]
    min_score: Dict[str, float] = field(default_factory=dict)
    _table: Dict[int, Tuple[int, str]] = field(default_factory=dict, init=False, repr=False)

    @staticmethod
    def from_dict(cfg: Mapping) -> 'ClassMapper':
        classes = [str(c) for c in cfg.get('classes', [])]
        name_map = {str(k): str(v) for k, v in (cfg.get('model_class_map') or {}).items()}
        for src, dst in name_map.items():
            if dst not in classes:
                raise ValueError(f'model_class_map[{src!r}] = {dst!r} 가 classes 에 없다')
        min_score = {str(k): float(v) for k, v in (cfg.get('min_score') or {}).items()}
        return ClassMapper(classes, name_map, min_score)

    @staticmethod
    def from_yaml(path: str) -> 'ClassMapper':
        with open(path, 'r', encoding='utf-8') as f:
            return ClassMapper.from_dict(yaml.safe_load(f) or {})

    def bind(self, model_names: Mapping[int, str]) -> Dict[int, Tuple[int, str]]:
        """모델의 names(dict id→이름) 에 대해 조회표를 만든다. 반환: 매핑되는 모델 id 표."""
        self._table = {}
        for mid, mname in model_names.items():
            dst = self.name_map.get(str(mname))
            if dst is not None:
                self._table[int(mid)] = (self.classes.index(dst), dst)
        return dict(self._table)

    def model_ids(self) -> List[int]:
        """추론에서 남길 모델 클래스 id (ultralytics classes= 인자)."""
        return sorted(self._table)

    def map(self, model_id: int, score: float) -> Optional[Tuple[int, str]]:
        """모델 id → (출력 id, 이름). 매핑 없음 또는 클래스별 최소 점수 미만이면 None."""
        hit = self._table.get(int(model_id))
        if hit is None:
            return None
        if score < self.min_score.get(hit[1], 0.0):
            return None
        return hit
