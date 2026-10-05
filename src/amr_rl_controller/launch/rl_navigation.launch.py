"""
RLController 를 얹은 navigation 스택 — 기존 런치·nav2_params.yaml 을 고치지 않는다.

    ros2 launch amr_rl_controller rl_navigation.launch.py model_path:=/p/policy_c.onnx mode:=C
    ros2 launch amr_rl_controller rl_navigation.launch.py model_path:=... replace_dwa:=true
    ros2 launch amr_rl_controller rl_navigation.launch.py model_path:=... sim:=kinematic

params_file(기본 amr_navigation config/nav2_params.yaml)을 읽어 amr_rl_controller.params 로
RL 블록을 덧씌운 임시 파일을 만들고, 그것을 params_file 로 넘겨 amr_navigation 의
navigation.launch.py (sim:=none, 기본) 또는 kinematic_sim.launch.py (sim:=kinematic, Gazebo 없는
폐루프) 를 포함한다. replace_dwa:=true 면 BT 기본 제어기 'DWA' 자리를 RL 이 대신한다.
포함한 런치의 인자(robot_name, prefix, use_sim_time, config_dir, x0, y0, yaw0, map,
moving_obstacles …)는 명령줄에 준 것만 넘긴다.
"""
import os

from amr_rl_controller.params import write_overlay
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PASS_NAV = ('robot_name', 'prefix', 'use_sim_time', 'config_dir', 'autostart', 'use_ttc_bt',
            'log_level')
PASS_KIN = ('robot_name', 'prefix', 'use_ttc_bt', 'log_level', 'x0', 'y0', 'yaw0',
            'moving_obstacles', 'cmd_topic', 'map')


def _setup(context):
    nav_share = get_package_share_directory('amr_navigation')
    get = lambda k: LaunchConfiguration(k, default='').perform(context)  # noqa: E731
    src = get('params_file') or os.path.join(nav_share, 'config', 'nav2_params.yaml')
    overrides = {}
    if get('max_inference_ms'):
        overrides['max_inference_ms'] = float(get('max_inference_ms'))
    if get('dwa_fallback'):
        overrides['dwa_fallback'] = get('dwa_fallback').lower() == 'true'
    path, cid = write_overlay(src, get('model_path'), get('mode'), get('controller_id'),
                              get('replace_dwa').lower() == 'true', overrides)
    sim = get('sim')
    if sim == 'kinematic':
        launch = os.path.join(nav_share, 'launch', 'kinematic_sim.launch.py')
        args = {k: get(k) for k in PASS_KIN if get(k) != ''}
    elif sim == 'none':
        launch = os.path.join(nav_share, 'launch', 'navigation.launch.py')
        args = {k: get(k) for k in PASS_NAV if get(k) != ''}
    else:
        raise RuntimeError(f"sim 은 none 또는 kinematic: {sim!r}")
    args['params_file'] = path
    return [
        LogInfo(msg=f'[rl_navigation] controller_id={cid} mode={get("mode")} '
                    f'model={get("model_path")} params={path} sim={sim}'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch),
                                 launch_arguments=args.items()),
    ]


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('model_path', description='정책 ONNX 경로 (필수)'),
        DeclareLaunchArgument('mode', default_value='C', description='B | C'),
        DeclareLaunchArgument('controller_id', default_value='RL'),
        DeclareLaunchArgument('replace_dwa', default_value='false'),
        DeclareLaunchArgument('max_inference_ms', default_value=''),
        DeclareLaunchArgument('dwa_fallback', default_value=''),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('sim', default_value='none', description='none | kinematic'),
        # 포함한 런치의 인자(robot_name, prefix, use_sim_time, config_dir, x0, map …)는 여기서 선언하지
        # 않는다: 포함한 런치가 부모 범위의 launch 인자를 그대로 보므로 빈 기본값을 선언하면 그쪽 기본값이
        # 가려진다 (amr_bringup launch_utils 의 map_yaml 주석과 같은 함정). 명령줄로 준 값만 넘긴다.
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
