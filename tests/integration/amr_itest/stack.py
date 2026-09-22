"""
시나리오별 launch 구성 (components.md §3–§5 의 노드 계약 기준).

두 가지 구성(profile)
  component  하네스가 노드 단위로 조립한다. 구성요소마다 실제 노드가 설치돼 있으면 실제 노드,
             없으면 대역(amr_itest.standins)을 쓴다 (ITEST_STANDINS 정책). 어느 쪽을 썼는지는
             result.json 의 components 에 남는다.
  system     amr_bringup/launch/system.launch.py 를 그대로 포함한다 (모든 하위 런치 필요).

백엔드(backend)
  kinematic  kinematic_sim 대역 + wall clock (GPU 불필요, 빠르고 결정적)
  gazebo     amr_simulation/warehouse.launch.py (헤드리스 Gazebo + 스폰 + 브리지) + sim time

속도 명령 체인 (components.md §4.1): cmd_vel_nav → velocity_profiler_node → cmd_vel_smoothed →
safety_node → cmd_vel. 체인을 넣은 스택에서 시나리오가 주행 명령을 낼 토픽은 drive_topic 이다.
"""

import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

from amr_itest import config
from amr_itest import requirements as req
from amr_itest.scenario import Context, GAZEBO, KINEMATIC
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from launch_ros.actions import Node

REAL = 'real'
STANDIN = 'standin'
EXTERNAL = 'external'     # 백엔드가 이미 제공 (예: Gazebo 가 imu/data 를 직접 발행)

# 실제 노드 목록: 구성요소 → (패키지, 실행 파일, 계약 근거)
REAL_NODES: Dict[str, Tuple[str, str, str]] = {
    'wheel_odometry': ('amr_localization', 'wheel_odometry_node', 'components.md §5.2'),
    'imu_filter': ('amr_localization', 'imu_filter_node', 'components.md §5.2'),
    'scan_filter': ('amr_localization', 'scan_filter_node', 'components.md §5.2'),
    'velocity_profiler': ('amr_navigation', 'velocity_profiler_node', 'components.md §5.3'),
    'safety': ('amr_perception', 'safety_node', 'components.md §5.4'),
}

# 실제 노드에 넘길 패키지 설정 후보 (있는 것만, components.md §6 설정 파일 매핑)
PACKAGE_CONFIGS: Dict[str, Tuple[str, ...]] = {
    'amr_localization': ('wheel_odometry.yaml', 'imu_filter.yaml', 'scan_filter.yaml',
                         'localization.yaml'),
    'amr_navigation': ('velocity_profiler.yaml',),
    'amr_perception': ('perception.yaml', 'safety.yaml'),
}

# system.launch.py 가 포함하는 하위 런치 (amr_bringup 계약)
SYSTEM_SUBLAUNCH = {
    'use_simulation': ('amr_simulation', 'warehouse.launch.py'),
    'use_localization': ('amr_localization', 'localization.launch.py'),
    'use_navigation': ('amr_navigation', 'navigation.launch.py'),
    'use_perception': ('amr_perception', 'perception.launch.py'),
    'use_behavior': ('amr_behavior', 'behavior.launch.py'),
}


def _bool(v: bool) -> str:
    return 'true' if v else 'false'


def system_requirements(use_simulation: bool = True, use_localization: bool = True,
                        use_navigation: bool = True, use_perception: bool = True,
                        use_behavior: bool = False) -> List[req.Requirement]:
    """
    system.launch.py 와 켜는 하위 런치의 요구사항.

    스켈레톤 시나리오는 Stack 을 만들기 전에 이것과 자기 노드 요구사항을 한 번에 확인해, 빠진
    것을 모두 건너뛰기 사유에 담는다 (패키지가 머지되면 자동으로 켜진다).
    """
    flags = {'use_simulation': use_simulation, 'use_localization': use_localization,
             'use_navigation': use_navigation, 'use_perception': use_perception,
             'use_behavior': use_behavior}
    reqs = [req.launch('amr_bringup', 'system.launch.py', 'system profile')]
    for flag, on in flags.items():
        if on:
            pkg, launch_file = SYSTEM_SUBLAUNCH[flag]
            reqs.append(req.launch(pkg, launch_file, f'system.launch.py {flag}:=true'))
    return reqs


class Stack:
    """시나리오 하나의 launch 엔티티 모음 + 구성요소별 실제/대역 기록."""

    def __init__(self, ctx: Context, backend: str, profile: str = 'component'):
        self.ctx = ctx
        self.settings = ctx.settings
        self.backend = backend
        self.profile = profile
        self.ns = self.settings.robot
        self.prefix = self.settings.frame_prefix
        self.use_sim_time = backend == GAZEBO
        self.entities: List = []
        self.components: Dict[str, str] = {}
        self.drive_topic = 'cmd_vel'
        self.cfg = config.config_dir()
        self.standin_names: List[str] = []
        self._pythonpath = os.pathsep.join(
            [str(config.HARNESS_DIR)] + [p for p in os.environ.get('PYTHONPATH', '').split(
                os.pathsep) if p])

    # ------------------------------------------------------------------ 공통
    def choose(self, component: str, reqs: Sequence[req.Requirement]) -> str:
        """
        실제/대역 결정 (ITEST_STANDINS).

        auto: 요구사항 충족 → real, 아니면 standin. always: 항상 standin.
        never: 요구사항이 없으면 시나리오를 건너뛴다.
        """
        policy = self.settings.standins
        missing = req.missing(reqs)
        if policy == 'always':
            kind = STANDIN
        elif not missing:
            kind = REAL
        elif policy == 'never':
            self.ctx.skip(f'{component}: real node required (ITEST_STANDINS=never) — '
                          + '; '.join(missing))
        else:
            kind = STANDIN
        self.components[component] = kind
        return kind

    def base_params(self) -> list:
        """모든 노드에 주는 설정: robot_params.yaml, sensors.yaml + 공통 오버라이드."""
        return [str(self.cfg / 'robot_params.yaml'), str(self.cfg / 'sensors.yaml'),
                {'use_sim_time': self.use_sim_time}]

    def standin(self, module: str, name: str, params: Optional[dict] = None,
                remappings: Optional[list] = None, with_config: bool = True) -> Node:
        """대역 노드 (python3 -m amr_itest.standins.<module>) 를 네임스페이스에 띄운다."""
        overrides = {'use_sim_time': self.use_sim_time, 'frame_prefix': self.prefix,
                     'seed': self.settings.seed}
        overrides.update(params or {})
        node = Node(executable=sys.executable,
                    arguments=['-m', f'amr_itest.standins.{module}'],
                    name=name, namespace=self.ns, exec_name=f'standin_{name}',
                    parameters=(self.base_params()[:2] if with_config else []) + [overrides],
                    remappings=remappings or [], output='screen',
                    additional_env={'PYTHONPATH': self._pythonpath})
        self.entities.append(node)
        self.standin_names.append(f'standin_{name}')
        return node

    def real_node(self, package: str, executable: str, name: Optional[str] = None,
                  params: Optional[list] = None, remappings: Optional[list] = None) -> Node:
        """설치된 실제 노드 (package 설정 파일이 있으면 함께 넘긴다)."""
        extra = []
        for fname in PACKAGE_CONFIGS.get(package, ()):
            path = req.config_file(package, fname)
            if path is not None:
                extra.append(str(path))
        node = Node(package=package, executable=executable, name=name or executable,
                    namespace=self.ns, output='screen',
                    parameters=self.base_params()[:2] + extra + (params or [])
                    + [{'use_sim_time': self.use_sim_time}],
                    remappings=remappings or [])
        self.entities.append(node)
        return node

    def include(self, package: str, launch_file: str, args: Dict[str, str]) -> None:
        path = req.launch_file(package, launch_file)
        if path is None:
            self.ctx.skip(f'launch file {package}/launch/{launch_file} not found')
        self.entities.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)), launch_arguments=list(args.items())))

    # ------------------------------------------------------------------ 시뮬레이터
    def simulator(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        """
        백엔드 + 센서 전처리 (imu/scan 필터는 실제 노드가 있으면 실제).

        kinematic: kinematic_sim 대역이 imu/data_raw·scan 을 내고, 필터가 없으면 imu/data·
                   scan_filtered 도 직접 낸다. 실제 imu_filter_node 에는 정지 바이어스 추정
                   시간(bias_estimation_time)을 2 s 로 줄여 준다 (시나리오 시작 직후 정지 구간).
        gazebo:    warehouse.launch.py 가 월드·스폰·브리지. 필터가 없으면 imu_filter·
                   topic_relay 대역.
        """
        sens = config.sensors()
        if self.backend == KINEMATIC:
            self.components['simulator'] = STANDIN
            imu_real = self.choose('imu_filter', [self._real_req('imu_filter')]) == REAL
            scan_real = self.choose('scan_filter', [self._real_req('scan_filter')]) == REAL
            self.standin('kinematic_sim', 'kinematic_sim', {
                'initial_x': x, 'initial_y': y, 'initial_yaw': yaw,
                'publish_corrected_imu': not imu_real,
                'scan_topics': ['scan'] if scan_real else ['scan', 'scan_filtered'],
            })
            if imu_real:
                self.real_node(*REAL_NODES['imu_filter'][:2],
                               params=[{'bias_estimation_time': 2.0}])
            if scan_real:
                self.real_node(*REAL_NODES['scan_filter'][:2])
            return
        # Gazebo
        self.components['simulator'] = REAL
        self.include('amr_simulation', 'warehouse.launch.py', {
            'headless': 'true', 'world': self.settings.world, 'use_sim_time': 'true',
            'spawn_robot': 'true', 'robot_name': self.ns,
            'x': str(x), 'y': str(y), 'yaw': str(yaw)})
        self.components['description'] = REAL
        # 브리지는 sensors.yaml imu.topic 으로 발행한다. 그 이름이 필터 출력(imu/data)이면
        # 시뮬레이터가 곧 필터 자리이므로 imu_filter_node 를 띄우지 않는다 (같은 토픽 이중 발행 방지).
        if config.get(sens, 'imu.topic', 'imu/data') == 'imu/data':
            self.components['imu_filter'] = EXTERNAL
        elif self.choose('imu_filter', [self._real_req('imu_filter')]) == REAL:
            self.real_node(*REAL_NODES['imu_filter'][:2], params=[{'bias_estimation_time': 2.0}])
        else:
            self.standin('imu_filter', 'imu_filter_node',
                         {'input_topic': config.get(sens, 'imu.topic', 'imu/data_raw')},
                         with_config=False)
        if self.choose('scan_filter', [self._real_req('scan_filter')]) == REAL:
            self.real_node(*REAL_NODES['scan_filter'][:2])
        else:
            self.standin('topic_relay', 'scan_filter_node', {
                'msg_type': 'sensor_msgs/msg/LaserScan', 'in_topic': 'scan',
                'out_topic': 'scan_filtered'}, with_config=False)

    def _real_req(self, component: str) -> req.Requirement:
        pkg, exe, why = REAL_NODES[component]
        return req.executable(pkg, exe, why)

    def description(self) -> None:
        """robot_state_publisher (URDF) 또는 static_tf 대역. Gazebo 백엔드는 스폰 런치가 포함."""
        if self.backend == GAZEBO:
            return
        kind = self.choose('description', [req.launch('amr_description',
                                                      'description.launch.py')])
        if kind == REAL:
            args = {'robot_name': self.ns, 'use_sim_time': _bool(self.use_sim_time),
                    'config_dir': str(self.cfg)}
            if self.prefix:
                args['prefix'] = self.prefix
            self.include('amr_description', 'description.launch.py', args)
        else:
            self.standin('static_tf', 'static_tf')

    # ------------------------------------------------------------------ 위치 추정
    def localization(self, amcl: bool = True, ekf: bool = True) -> None:
        """
        wheel_odometry(실제/대역) + amcl 대역 + robot_localization 이중 EKF (config/ekf.yaml).

        실제 AMCL 은 Gazebo 스캔과 저장된 지도가 필요해 system 프로필에서만 쓴다.
        amcl=False 면 map EKF 도 빠진다 (amcl_pose 없이는 map → odom 이 의미 없다).
        ekf=False 면 휠 오도메트리만 (safety_node 의 엔코더 생존 감시용).
        """
        if self.choose('wheel_odometry', [self._real_req('wheel_odometry')]) == REAL:
            self.real_node(*REAL_NODES['wheel_odometry'][:2])
        else:
            self.standin('wheel_odometry', 'wheel_odometry_node')
        if not ekf:
            return
        if amcl:
            # 대역 AMCL 노이즈: 축별 σ 1.5 cm · yaw 0.01 rad, 2 Hz, 지연 0.1 s. config/ekf.yaml 의
            # map 필터는 x·y 프로세스 노이즈가 커서(0.05) AMCL 고정값을 거의 그대로 따르므로 정지 구간
            # RMSE ≈ √2·σ 가 된다 (시나리오 04 결과 해석 시 참고 — 실제 AMCL 이 들어오면 그 값을 잰다).
            self.components['amcl'] = STANDIN
            self.standin('amcl_pose', 'amcl', {'rate': 2.0, 'sigma_xy': 0.015,
                                               'sigma_yaw': 0.01, 'latency': 0.1})
        self.ctx.require([req.executable('robot_localization', 'ekf_node',
                                         'dual EKF (config/ekf.yaml)')])
        self.components['ekf'] = REAL
        ekf_yaml = str(self.cfg / 'ekf.yaml')
        frames = {'use_sim_time': self.use_sim_time}
        if self.prefix:
            frames.update({'odom_frame': self.prefix + 'odom',
                           'base_link_frame': self.prefix + 'base_footprint'})
        for name, remap in (('ekf_filter_node_odom', []),
                            ('ekf_filter_node_map',
                             [('odometry/filtered', 'odometry/filtered_map')])):
            if name == 'ekf_filter_node_map' and not amcl:
                continue
            self.entities.append(Node(
                package='robot_localization', executable='ekf_node', name=name,
                namespace=self.ns, output='screen', parameters=[ekf_yaml, frames],
                remappings=remap))

    # ------------------------------------------------------------------ 속도 명령 체인
    def velocity_chain(self, profiler: bool = True, safety: bool = True) -> None:
        """cmd_vel_nav → (profiler) → cmd_vel_smoothed → (safety) → cmd_vel."""
        if profiler:
            if self.choose('velocity_profiler', [self._real_req('velocity_profiler')]) == REAL:
                self.real_node(*REAL_NODES['velocity_profiler'][:2])
            else:
                self.standin('cmd_relay', 'velocity_profiler_node')
            self.drive_topic = 'cmd_vel_nav'
        if safety:
            if self.choose('safety', [self._real_req('safety')]) == REAL:
                self.real_node(*REAL_NODES['safety'][:2])
            else:
                self.standin('safety_gate', 'safety_node')
            if not profiler:
                self.drive_topic = 'cmd_vel_smoothed'

    def task_executor(self, drive_speed: float = 0.3, drive_time: float = 0.6) -> None:
        """
        assign_task 서버. component 프로필에서는 항상 대역.

        실제 task_executor_node 는 navigate_to_pose(Nav2) + 지도 + 위치 추정이 있어야 동작하므로
        system 프로필(system.launch.py use_behavior:=true)에서 시험한다.
        """
        if self.settings.standins == 'never':
            self.ctx.skip('task_executor: the real node needs the full Nav2 system — '
                          'run with ITEST_PROFILE=system (ITEST_STANDINS=never)')
        self.components['task_executor'] = STANDIN
        self.standin('task_executor', 'task_executor_node',
                     {'drive_speed': drive_speed, 'drive_time': drive_time,
                      'robot_id': self.ns}, with_config=False)

    # ------------------------------------------------------------------ 평가 도구
    def eval_logger(self, executable: str, params: dict, name: Optional[str] = None) -> bool:
        """
        amr_evaluation 로거 (feature/evaluation-tools) 를 시나리오 로그 디렉토리에 기록하게 띄운다.

        CSV 는 logs/itest/<시나리오 id>/ 에 쌓인다 (output_dir = 로그 루트, run_name = id).
        amr_evaluation 이 없으면 띄우지 않고 False — 시나리오는 하네스 자체 계산(metrics.py)만으로
        판정한다 (result.json components.evaluation = 'harness-only').
        """
        if not req.has_executable('amr_evaluation', executable):
            self.components['evaluation'] = 'harness-only'
            return False
        cfg = req.config_file('amr_evaluation', 'amr_evaluation.yaml')
        overrides = {'output_dir': str(self.settings.log_root), 'run_name': self.ctx.scenario.id,
                     'use_sim_time': self.use_sim_time, 'report_period': 0.0}
        overrides.update(params)
        self.components['evaluation'] = REAL
        self.entities.append(Node(
            package='amr_evaluation', executable=executable, name=name or executable,
            namespace=self.ns, output='screen',
            parameters=([str(cfg)] if cfg else []) + [overrides]))
        return True

    # ------------------------------------------------------------------ system 프로필
    def system(self, use_localization: bool = True, use_navigation: bool = True,
               use_perception: bool = True, use_behavior: bool = False,
               extra_args: Optional[Dict[str, str]] = None) -> None:
        """amr_bringup system.launch.py 를 포함한다 (필요한 하위 런치를 먼저 확인)."""
        flags = {'use_simulation': self.backend == GAZEBO, 'use_localization': use_localization,
                 'use_navigation': use_navigation, 'use_perception': use_perception,
                 'use_behavior': use_behavior}
        self.ctx.require(system_requirements(**flags), 'system profile')
        args = {'use_sim_time': _bool(self.use_sim_time), 'robot_name': self.ns}
        args.update({k: _bool(v) for k, v in flags.items()})
        args.update(extra_args or {})
        self.include('amr_bringup', 'system.launch.py', args)
        for flag, on in flags.items():
            self.components[flag.replace('use_', '')] = REAL if on else 'off'
        if use_navigation:
            self.drive_topic = 'cmd_vel_nav'

    # ------------------------------------------------------------------ 마무리
    def add(self, entity) -> None:
        self.entities.append(entity)

    def launch_description(self) -> LaunchDescription:
        """
        맨 앞에 ReadyToTest 를 둔 LaunchDescription.

        launch_testing 은 ReadyToTest 를 15 s 안에 못 보면 테스트를 포기한다. 부하가 큰 호스트에서
        xacro 전개·Gazebo 기동이 그보다 오래 걸리므로 준비 판정은 테스트 쪽(토픽 대기)에 맡긴다.
        """
        self.ctx.record.set_components(self.components)
        return LaunchDescription([launch_testing.actions.ReadyToTest()] + self.entities)

    def summary(self) -> str:
        return ', '.join(f'{k}={v}' for k, v in sorted(self.components.items()))
