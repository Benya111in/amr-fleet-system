"""
RViz2 설정이 실제 발행 토픽과 어긋나지 않게 고정한다 (명세 9장 552행).

설정 파일은 실행해 보지 않으면 조용히 낡는다. 이 서버에는 DISPLAY 가 없어 RViz2 를 띄울 수
없으므로(시나리오 15 머리말이 같은 이유로 토픽 판정을 쓴다), 최소한 **설정이 가리키는 토픽이
코드·설정이 실제로 발행하는 것인지**는 시험으로 묶는다.
"""
import re
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
RVIZ = REPO / 'src/amr_bringup/rviz/perception.rviz'


def _topics(cfg):
    """설정에서 Topic.Value / Description Topic.Value 를 전부 걷는다."""
    out = []

    def walk(node):
        if isinstance(node, dict):
            for k, v in node.items():
                if k in ('Topic', 'Description Topic') and isinstance(v, dict) and 'Value' in v:
                    out.append(v['Value'])
                else:
                    walk(v)
        elif isinstance(node, list):
            for x in node:
                walk(x)
    walk(cfg)
    return out


def test_rviz_config_parses():
    assert RVIZ.is_file(), f'{RVIZ} 가 없다'
    cfg = yaml.safe_load(RVIZ.read_text(encoding='utf-8'))
    assert cfg.get('Visualization Manager'), 'Visualization Manager 블록이 없다'


def test_perception_marker_topic_is_displayed():
    """명세 4.6 의 핵심인 인지 마커 토픽이 반드시 들어 있어야 한다."""
    cfg = yaml.safe_load(RVIZ.read_text(encoding='utf-8'))
    tops = _topics(cfg)
    assert '/amr_01/perception/markers' in tops, tops
    # 시나리오 15 가 판정에 쓰는 토픽 이름과 같아야 한다 (상대 이름 + 네임스페이스)
    t15 = (REPO / 'tests/integration/test_15_perception_3d.py').read_text(encoding='utf-8')
    m = re.search(r"MARKERS_TOPIC\s*=\s*'([^']+)'", t15)
    assert m, 'test_15 에서 MARKERS_TOPIC 을 찾지 못했다'
    assert f'/amr_01/{m.group(1)}' in tops, \
        f'설정의 마커 토픽이 시나리오 15 의 {m.group(1)} 와 어긋난다'


@pytest.mark.parametrize('topic', ['/map', '/amr_01/scan', '/amr_01/plan'])
def test_core_topics_present(topic):
    cfg = yaml.safe_load(RVIZ.read_text(encoding='utf-8'))
    assert topic in _topics(cfg)


def test_installed_by_cmake():
    """share/ 에 설치되지 않으면 ros2 run 으로 못 쓴다."""
    cm = (REPO / 'src/amr_bringup/CMakeLists.txt').read_text(encoding='utf-8')
    block = re.search(r'install\(DIRECTORY(.*?)DESTINATION', cm, re.S)
    assert block and 'rviz' in block.group(1), 'CMakeLists 가 rviz 디렉터리를 설치하지 않는다'
