"""
기본 구성 무변화 증명 (보너스 2 — 3D LiDAR 를 넣어도 기본 5대·기존 시나리오의 모델과 브리지가 그대로다).

- xacro 기본 인자(lidar3d 생략)와 lidar3d:=false 의 URDF 가 d8ae80c(3D LiDAR 이전) 의 출력과 **바이트 단위로** 같다.
  변형: 단일 로봇 기본, multi_robot 기본 5대(amr_0N, 접두어 amr_0N/), 적재 large, runtime_payload false.
  설정은 golden/config/ 의 고정 사본을 쓴다 → 최상위 config/ 가 바뀌어도 이 시험은 xacro 변경만 본다.
- spawn.launch.py 가 만드는 parameter_bridge 항목도 d8ae80c 와 같다 (lidar3d 없음).
- 3D LiDAR 로봇의 URDF 는 기본과 "IMU 주기·σ" 와 "끝에 붙은 3D LiDAR 블록" 만 다르다.
골든을 다시 만드는 법은 golden/make_golden.py 머리말.
"""

import difflib
import importlib.util
from pathlib import Path
import re
import sys

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
GOLDEN = PKG / 'test' / 'golden'
sys.path.insert(0, str(GOLDEN))
import make_golden as mg  # noqa: E402

XACRO = PKG / 'urdf' / 'amr.urdf.xacro'


def _xacro(args: dict) -> str:
    return mg.run_xacro(XACRO, GOLDEN / 'config', args)


def _diff(a: str, b: str) -> str:
    return ''.join(list(difflib.unified_diff(a.splitlines(True), b.splitlines(True)))[:60])


@pytest.mark.parametrize('name, args', mg.VARIANTS, ids=[v[0] for v in mg.VARIANTS])
@pytest.mark.parametrize('lidar3d', [None, 'false'])
def test_default_urdf_byte_identical(name, args, lidar3d):
    gold = (GOLDEN / f'{name}.urdf').read_bytes()
    extra = {} if lidar3d is None else {'lidar3d': lidar3d}
    out = _xacro({**args, **extra}).encode('utf-8')
    assert out == gold, _diff(gold.decode('utf-8'), out.decode('utf-8'))


def test_golden_has_no_lidar3d():
    """골든 자체가 3D LiDAR 이전 것인지 (실수로 새 출력으로 덮어쓰지 않았는지)."""
    for path in GOLDEN.glob('*.urdf'):
        text = path.read_text(encoding='utf-8')
        assert 'lidar3d' not in text, path.name
        assert '<update_rate>100.0</update_rate>' in text      # IMU 기본 100 Hz


def _spawn_module():
    spec = importlib.util.spec_from_file_location('spawn_launch_golden',
                                                  PKG / 'launch' / 'spawn.launch.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.parametrize('odom_tf, name', [(False, 'bridge_default.yaml'),
                                           (True, 'bridge_odom_tf.yaml')])
def test_default_bridge_entries_unchanged(odom_tf, name):
    pytest.importorskip('launch')
    spawn = _spawn_module()
    sensors = yaml.safe_load((GOLDEN / 'config' / 'sensors.yaml').read_text(
        encoding='utf-8'))['/**']['ros__parameters']
    gold = yaml.safe_load((GOLDEN / name).read_text(encoding='utf-8'))
    assert spawn._bridge_entries(sensors, odom_tf) == gold
    assert spawn._bridge_entries(sensors, odom_tf, None) == gold


def test_lidar3d_robot_differs_only_in_imu_and_appended_block():
    """3D 로봇 URDF = 기본 URDF 에서 IMU 주기·σ 7 줄이 바뀌고, IMU 센서 뒤에 3D 블록이 붙은 것."""
    args = {'robot_name': 'amr_01', 'prefix': 'amr_01/'}
    gold = (GOLDEN / 'fleet_amr_01.urdf').read_text(encoding='utf-8')
    out = _xacro({**args, 'lidar3d': 'true'})
    sm = difflib.SequenceMatcher(a=gold.splitlines(), b=out.splitlines(), autojunk=False)
    changed, inserted = [], []
    for op, i1, i2, j1, j2 in sm.get_opcodes():
        if op == 'replace':
            changed += list(zip(gold.splitlines()[i1:i2], out.splitlines()[j1:j2]))
        elif op == 'insert':
            inserted.append(out.splitlines()[j1:j2])
        elif op == 'delete':
            pytest.fail(f'기본 URDF 의 줄이 사라졌다: {gold.splitlines()[i1:i2]}')
    assert len(changed) == 7                           # update_rate 1 + 축별 σ 6
    for old, new in changed:
        assert re.sub(r'>[0-9.e-]+<', '><', old) == re.sub(r'>[0-9.e-]+<', '><', new)
    assert len(inserted) == 1
    block = '\n'.join(inserted[0])
    assert 'lidar3d_link' in block and 'type="gpu_lidar"' in block
