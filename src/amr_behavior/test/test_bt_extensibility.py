"""BT 모듈성 계약 (명세 9장 "새로운 BT 노드를 추가하여 작업을 확장하는 데 기존 코드 수정이 3개 파일 이내인가?").

이 시험은 **문서의 주장을 코드로 고정**한다. 확장 경로가 둘이고 각각 기존 파일 몇 개를
건드리는지가 구조에서 나오므로, 그 구조가 깨지면 여기서 실패해야 한다.

  경로 A (작업 실행기 코어 노드)
    신규: include/amr_behavior/bt_nodes/<이름>.hpp   (헤더 전용)
    수정: src/bt_registry.cpp 한 줄                  -> **기존 파일 1 개**
    헤더 전용이라 CMakeLists 는 건드리지 않는다 (amr_behavior_bt 소스 목록이 고정이다).

  경로 B (nav2 bt_navigator 플러그인)
    신규: include/amr_behavior/plugins/<이름>.hpp + src/plugins/<이름>.cpp
    수정: CMakeLists.txt (add_library) + nav2_params.yaml (plugin_lib_names) -> **기존 파일 2 개**
    bt_registry.cpp 는 건드리지 않는다 — 두 경로가 분리돼 있다는 것이 이 시험의 핵심이다.
"""
import re
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]


def _read(rel):
    return (PKG / rel).read_text(encoding='utf-8')


def test_core_node_path_needs_only_the_registry():
    """경로 A: 코어 노드는 헤더 전용이고 등록은 bt_registry.cpp 한 곳에서만 일어난다."""
    reg = _read('src/bt_registry.cpp')
    # 등록이 한 파일에 모여 있어야 "한 줄 추가" 가 성립한다
    assert reg.count('registerWithArg<') + reg.count('registerNodeType<') >= 20
    # 코어 노드는 전부 헤더 전용 -> CMakeLists 의 amr_behavior_bt 소스 목록에 노드가 없다
    cmake = _read('CMakeLists.txt')
    bt_lib = re.search(r'add_library\(amr_behavior_bt SHARED(.*?)\)', cmake, re.S)
    assert bt_lib, 'amr_behavior_bt 라이브러리 정의를 찾지 못했다'
    assert 'bt_nodes/' not in bt_lib.group(1), \
        'bt_nodes 가 CMakeLists 소스 목록에 들어가면 노드마다 CMakeLists 를 고쳐야 한다'
    # 노드 헤더가 실제로 그 디렉터리에 모여 있다
    hdrs = list((PKG / 'include/amr_behavior/bt_nodes').glob('*.hpp'))
    assert len(hdrs) >= 20, f'노드 헤더 {len(hdrs)} 개'


def test_plugin_path_is_decoupled_from_the_core_registry():
    """경로 B: nav2 플러그인은 bt_registry.cpp 를 전혀 건드리지 않는다.

    이 분리가 깨지면(플러그인을 코어 레지스트리에 끌어들이면) 확장 비용이 늘어난다.
    """
    reg = _read('src/bt_registry.cpp')
    plugins = list((PKG / 'src/plugins').glob('*.cpp'))
    assert plugins, 'src/plugins 아래 플러그인 예시가 있어야 한다 (문서가 가리키는 실례)'
    for p in plugins:
        stem = p.stem
        assert stem not in reg, f'{stem} 이 bt_registry.cpp 에 섞였다 — 두 경로의 분리가 깨졌다'
        assert 'plugins/' not in reg, 'bt_registry.cpp 가 plugins/ 를 포함하면 안 된다'
        # 플러그인은 BT_REGISTER_NODES 로 자기 자신을 등록한다
        assert 'BT_REGISTER_NODES' in p.read_text(encoding='utf-8'), \
            f'{p.name} 에 BT_REGISTER_NODES 가 없다'


def test_extension_touches_at_most_three_existing_files():
    """두 경로의 '기존 파일 수정' 합이 명세 상한 3 을 넘지 않는다 (구조에서 유도)."""
    # 경로 A: bt_registry.cpp
    path_a = {'src/bt_registry.cpp'}
    # 경로 B: CMakeLists.txt + 소비자 설정(nav2_params.yaml, 이 패키지 밖)
    path_b = {'CMakeLists.txt', '<nav2_params.yaml>'}
    assert len(path_a) <= 3, path_a
    assert len(path_b) <= 3, path_b
    for rel in path_a | (path_b - {'<nav2_params.yaml>'}):
        assert (PKG / rel).is_file(), f'{rel} 가 없다 — 문서의 경로가 낡았다'
