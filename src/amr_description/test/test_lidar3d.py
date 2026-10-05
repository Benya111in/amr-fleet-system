"""
3D LiDAR 로봇 모델 (보너스 2, xacro lidar3d:=true) 과 amr_description.lidar3d 런치 보조 모듈 시험.

- 링크·조인트·시각체: lidar3d_link 가 base_link 의 고정 자식이고 광학 중심이 지면 +0.90 m, 마스트가 데크 상면에서 시작,
  모든 시각체에 자기 가시성 비트.
- gpu_lidar: 16 ch ±15°, 1800 열(0.2° 정확 증분), 10 Hz, 0.10~30 m, σ 0.03, 토픽 /<robot>/lidar3d,
  frame <prefix>lidar3d_link, 자기 차체 mask.
- IMU: 3D 로봇만 200 Hz, 가속도·자이로 이산 σ ×√2, 바이어스 그대로.
- 모듈: true|false|auto 해석, 스폰 목록에서 로봇별 키 읽기, 브리지 항목(점군만, lazy).
"""

import math
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from amr_description import lidar3d as l3d
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
CONFIG = PKG.parents[1] / 'config'
XACRO = PKG / 'urdf' / 'amr.urdf.xacro'
L3_PARAMS = l3d.load_config(str(PKG / 'config' / 'lidar3d.yaml'))
L3 = L3_PARAMS['lidar3d']
FULL_MASK = 0xFFFFFFFF


def _params(name):
    with open(CONFIG / name, encoding='utf-8') as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


ROBOT = _params('robot_params.yaml')['robot']
LARGE_H = _params('robot_params.yaml')['payload']['large']['size'][2]
IMU = _params('sensors.yaml')['imu']


def _xacro(**kw) -> ET.Element:
    args = ['xacro', str(XACRO), f'config_dir:={CONFIG}'] + [f'{k}:={v}' for k, v in kw.items()]
    r = subprocess.run(args, capture_output=True, text=True, check=False)
    assert r.returncode == 0, r.stderr
    return ET.fromstring(r.stdout)


@pytest.fixture(scope='module')
def l3_urdf():
    return _xacro(robot_name='amr_02', prefix='amr_02/', lidar3d='true')


@pytest.fixture(scope='module')
def base_urdf():
    return _xacro(robot_name='amr_02', prefix='amr_02/')


def _sensor(root, kind, name=None):
    found = [s for s in root.iter('sensor') if s.get('type') == kind
             and (name is None or s.get('name') == name)]
    assert len(found) == 1, (kind, name, len(found))
    return found[0]


def _f(elem, path):
    return float(elem.find(path).text)


def test_link_joint_and_height(l3_urdf):
    joint = l3_urdf.find("joint[@name='amr_02/lidar3d_joint']")
    assert joint is not None and joint.get('type') == 'fixed'
    assert joint.find('parent').get('link') == 'amr_02/base_link'
    assert joint.find('child').get('link') == 'amr_02/' + L3['frame_id']
    xyz = [float(v) for v in joint.find('origin').get('xyz').split()]
    ext = L3['extrinsic']
    assert xyz == pytest.approx([ext['x'], ext['y'], ext['z']])
    assert joint.find('origin').get('rpy').split() == ['0.0', '0.0', '0.0']
    # 광학 중심 지면 +0.90 m (보고서 장착 높이), 데크 앞끝 안쪽
    assert ROBOT['base_link_height'] + ext['z'] == pytest.approx(0.90)
    assert 0.0 < ext['x'] < ROBOT['footprint_length'] / 2.0
    # 대형 화물(상단 +0.73) 먼 모서리를 −15° 빔이 넘는다: H ≥ 0.73 + (화물 길이)·tan15°
    deck = ROBOT['base_link_height'] + ROBOT['height'] / 2.0
    far = ext['x'] + ROBOT['footprint_length'] / 2.0
    assert 0.90 >= deck + LARGE_H + far * math.tan(-L3['vertical_min'])


def test_mast_visuals(l3_urdf):
    link = l3_urdf.find(f"link[@name='amr_02/{L3['frame_id']}']")
    visuals = link.findall('visual')
    assert len(visuals) == 2
    assert all(v.get('name') is None for v in visuals)     # sdformat 가시성 짝짓기 규칙
    assert link.find('collision') is None and link.find('inertial') is None
    mast = visuals[1]
    length = float(mast.find('geometry/cylinder').get('length'))
    z = float(mast.find('origin').get('xyz').split()[2])
    bottom = L3['extrinsic']['z'] + z - length / 2.0       # base_link 기준
    top = L3['extrinsic']['z'] + z + length / 2.0
    assert bottom == pytest.approx(ROBOT['height'] / 2.0, abs=1e-6)   # 데크 상면
    assert top == pytest.approx(L3['extrinsic']['z'] - L3['head_length'] / 2.0, abs=1e-6)


def test_gpu_lidar_definition(l3_urdf):
    s = _sensor(l3_urdf, 'gpu_lidar', 'amr_02/lidar3d')
    assert s.find('topic').text == '/amr_02/' + L3['topic']
    assert s.find('ignition_frame_id').text == 'amr_02/' + L3['frame_id']
    assert _f(s, 'update_rate') == pytest.approx(10.0)
    h, v = s.find('lidar/scan/horizontal'), s.find('lidar/scan/vertical')
    assert int(h.find('samples').text) == 1800 and int(v.find('samples').text) == 16
    assert h.find('resolution').text == '1' and v.find('resolution').text == '1'
    inc_h = (_f(h, 'max_angle') - _f(h, 'min_angle')) / 1799
    assert math.degrees(inc_h) == pytest.approx(0.2, abs=1e-6)       # 첫·끝 열 겹침 없음
    assert _f(h, 'min_angle') == pytest.approx(-math.pi, abs=1e-7)
    assert math.degrees(_f(v, 'min_angle')) == pytest.approx(-15.0, abs=1e-4)
    assert math.degrees(_f(v, 'max_angle')) == pytest.approx(15.0, abs=1e-4)
    assert _f(s, 'lidar/range/min') == pytest.approx(0.10)
    assert _f(s, 'lidar/range/max') == pytest.approx(30.0)
    assert s.find('lidar/noise/type').text == 'gaussian'
    assert _f(s, 'lidar/noise/stddev') == pytest.approx(0.03)
    bit = 4 + (2 - 1)                                                # amr_02 → 비트 5
    assert int(s.find('lidar/visibility_mask').text) == FULL_MASK - (1 << bit)


def test_self_visibility_on_new_visuals(l3_urdf):
    refs = {g.get('reference') for g in l3_urdf.findall('gazebo')
            if g.find('visual/visibility_flags') is not None}
    assert f"amr_02/{L3['frame_id']}" in refs
    flags = {int(e.text) for e in l3_urdf.iter('visibility_flags')}
    assert flags == {1 << 5}


def _imu_values(root):
    s = _sensor(root, 'imu')
    rate = _f(s, 'update_rate')
    gyro = [_f(s, f'imu/angular_velocity/{a}/noise/stddev') for a in 'xyz']
    accel = [_f(s, f'imu/linear_acceleration/{a}/noise/stddev') for a in 'xyz']
    bias = [_f(s, f'imu/angular_velocity/{a}/noise/bias_mean') for a in 'xyz'] + \
        [_f(s, f'imu/linear_acceleration/{a}/noise/bias_mean') for a in 'xyz']
    return rate, gyro, accel, bias


def test_imu_rate_and_sigma(l3_urdf, base_urdf):
    rate, gyro, accel, bias = _imu_values(l3_urdf)
    rate0, gyro0, accel0, bias0 = _imu_values(base_urdf)
    assert rate0 == pytest.approx(IMU['update_rate'])
    assert rate == pytest.approx(L3_PARAMS['imu_update_rate']) == pytest.approx(200.0)
    k = math.sqrt(rate / rate0)
    assert gyro == pytest.approx([IMU['gyro_noise_stddev'] * k] * 3)
    assert accel == pytest.approx([IMU['accel_noise_stddev'] * k] * 3)
    assert gyro[0] == pytest.approx(2.83e-4, rel=1e-3)               # 보고서 값
    assert accel[0] == pytest.approx(0.024, rel=2e-3)
    assert gyro0 == pytest.approx([IMU['gyro_noise_stddev']] * 3)
    assert bias == bias0                                             # 바이어스는 주기와 무관
    # 방위는 끄지 않는다 (끄면 w=0 쿼터니언 — amr_gazebo.xacro IMU 주석). 소비자가 쓰지 않는다
    assert _sensor(l3_urdf, 'imu').find('imu/enable_orientation').text == 'true'


def test_other_sensors_untouched(l3_urdf, base_urdf):
    """2D LiDAR·카메라 정의는 3D 로봇에서도 같다."""
    for kind in ('camera', 'depth_camera'):
        a = ET.tostring(_sensor(l3_urdf, kind))
        b = ET.tostring(_sensor(base_urdf, kind))
        assert a == b, kind
    a = ET.tostring(_sensor(l3_urdf, 'gpu_lidar', 'amr_02/lidar'))
    assert a == ET.tostring(_sensor(base_urdf, 'gpu_lidar', 'amr_02/lidar'))


def test_single_root_tree(l3_urdf):
    links = {e.get('name') for e in l3_urdf.findall('link')}
    children = [j.find('child').get('link') for j in l3_urdf.findall('joint')]
    assert len(children) == len(set(children))
    assert links - set(children) == {'amr_02/base_footprint'}


def test_sdf_conversion_keeps_sensor_and_flags():
    """`ign sdf -p` 뒤에도 gpu_lidar 2개(2D·3D)와 모든 시각체의 자기 비트가 남는다 (도구 없으면 건너뜀)."""
    exe = shutil.which('ign')
    if exe is None:
        pytest.skip('ign 없음')
    r = subprocess.run(['xacro', str(XACRO), f'config_dir:={CONFIG}', 'robot_name:=amr_01',
                        'prefix:=amr_01/', 'lidar3d:=true'], capture_output=True, text=True,
                       check=True)
    with tempfile.NamedTemporaryFile('w', suffix='.urdf') as f:
        f.write(r.stdout)
        f.flush()
        out = subprocess.run([exe, 'sdf', '-p', f.name], capture_output=True, text=True,
                             check=False)
    assert out.returncode == 0, out.stderr
    sdf = ET.fromstring(out.stdout)
    lidars = [s for s in sdf.iter('sensor') if s.get('type') == 'gpu_lidar']
    assert sorted(s.get('name') for s in lidars) == ['amr_01/lidar', 'amr_01/lidar3d']
    l3 = next(s for s in lidars if s.get('name') == 'amr_01/lidar3d')
    # 병합 뒤 센서 자세 = base_footprint 기준 (x, 0, 0.90)
    pose = [float(v) for v in l3.find('pose').text.split()]
    assert pose[:3] == pytest.approx([L3['extrinsic']['x'], 0.0, 0.90], abs=1e-6)
    for v in sdf.iter('visual'):
        assert {int(e.text) for e in v.findall('visibility_flags')} == {1 << 4}, v.get('name')


# ---------------------------------------------------------------- 모듈 (launch 비의존)

@pytest.mark.parametrize('text, expected', [
    ('true', True), ('True', True), ('1', True), ('yes', True),
    ('false', False), ('0', False), ('no', False), ('auto', None), ('', None),
])
def test_parse_flag(text, expected):
    assert l3d.parse_flag(text) is expected


def test_parse_flag_rejects():
    with pytest.raises(ValueError):
        l3d.parse_flag('maybe')


def _robots_file(tmp_path, robots):
    path = tmp_path / 'spawn.yaml'
    path.write_text(yaml.safe_dump({'robots': robots}), encoding='utf-8')
    return str(path)


def test_resolve(tmp_path):
    f = _robots_file(tmp_path, [{'name': 'amr_01', 'x': 0, 'y': 0, 'yaw': 0, 'lidar3d': True},
                                {'name': 'amr_02', 'x': 2, 'y': 0, 'yaw': 0}])
    assert l3d.resolve('auto', 'amr_01', f) is True
    assert l3d.resolve('auto', 'amr_02', f) is False            # 키 없음
    assert l3d.resolve('auto', 'amr_09', f) is False            # 항목 없음
    assert l3d.resolve('auto', 'amr_01', '') is False           # 스폰 목록 없음
    assert l3d.resolve('auto', 'amr_01', None) is False
    assert l3d.resolve('false', 'amr_01', f) is False           # 명시 값이 이긴다
    assert l3d.resolve('true', 'amr_02', f) is True


def test_resolve_rejects_non_bool(tmp_path):
    f = _robots_file(tmp_path, [{'name': 'amr_01', 'x': 0, 'y': 0, 'yaw': 0, 'lidar3d': 'yes'}])
    with pytest.raises(ValueError, match='lidar3d'):
        l3d.resolve('auto', 'amr_01', f)


def test_bridge_entries_points_only_lazy():
    entries = l3d.bridge_entries(L3_PARAMS)
    assert entries == [{
        'ros_topic_name': 'lidar3d/points_raw', 'gz_topic_name': 'lidar3d/points',
        'ros_type_name': 'sensor_msgs/msg/PointCloud2',
        'gz_type_name': 'ignition.msgs.PointCloudPacked',
        'direction': 'GZ_TO_ROS', 'lazy': True}]
    assert L3['points_topic'] != L3['points_raw_topic']


def test_load_config_rejects_missing_block(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'imu_update_rate': 200.0}}}),
                    encoding='utf-8')
    with pytest.raises(ValueError):
        l3d.load_config(str(path))
    assert l3d.default_config_path('/x').endswith('/x/config/lidar3d.yaml')
