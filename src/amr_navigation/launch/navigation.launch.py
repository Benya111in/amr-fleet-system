"""
Nav2 + 직접 구현 계획기/제어기 + velocity_profiler_node 기동 (components.md §3.3, multi_robot.md §4).

    ros2 launch amr_navigation navigation.launch.py            # 단일 로봇 (ns amr_01, 접두어 없음)
    ros2 launch amr_navigation navigation.launch.py robot_name:=amr_02 prefix:=amr_02/

- 모든 노드는 robot_name 네임스페이스 아래에서 뜨고 토픽은 상대 이름이다. /tf, /tf_static 은 리맵하지 않는다
  (공유 TF 설계 — nav2_bringup 의 navigation_launch.py 는 /tf 를 리맵하므로 포함하지 않는다).
- 파라미터 템플릿(config/nav2_params.yaml)의 '<prefix>' 를 ReplaceString 으로 프레임 접두어로,
  '<robot_ns>' 를 '/<robot_name>' 으로 바꾸고 (코스트맵 레이어 토픽은 costmap 자식 네임스페이스에서
  풀리므로 절대 이름이 필요하다), RewrittenYaml(root_key=robot_name) 으로 네임스페이스 키를 씌운다.
- controller_server / behavior_server 의 cmd_vel 은 cmd_vel_nav 로 리맵 → velocity_profiler_node →
  cmd_vel_smoothed → safety_node → cmd_vel (components.md §4.1).
- BT: amr_behavior 의 IsTTCBelowThreshold 플러그인이 설치돼 있으면 TTC 조건 포함 BT, 아니면 대체 BT
  (use_ttc_bt:=auto|true|false).
"""
import os

from ament_index_python.packages import (get_package_prefix, get_package_share_directory,
                                         PackageNotFoundError)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml

PKG = 'amr_navigation'
TTC_BT_LIB = 'amr_is_ttc_below_threshold_condition_bt_node'
LIFECYCLE_NODES = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
NAV2_BT_LIBS = [
    'nav2_compute_path_to_pose_action_bt_node',
    'nav2_compute_path_through_poses_action_bt_node',
    'nav2_follow_path_action_bt_node',
    'nav2_spin_action_bt_node',
    'nav2_wait_action_bt_node',
    'nav2_back_up_action_bt_node',
    'nav2_drive_on_heading_bt_node',
    'nav2_clear_costmap_service_bt_node',
    'nav2_is_stuck_condition_bt_node',
    'nav2_goal_reached_condition_bt_node',
    'nav2_goal_updated_condition_bt_node',
    'nav2_globally_updated_goal_condition_bt_node',
    'nav2_is_path_valid_condition_bt_node',
    'nav2_initial_pose_received_condition_bt_node',
    'nav2_rate_controller_bt_node',
    'nav2_distance_controller_bt_node',
    'nav2_speed_controller_bt_node',
    'nav2_truncate_path_action_bt_node',
    'nav2_truncate_path_local_action_bt_node',
    'nav2_goal_updater_node_bt_node',
    'nav2_recovery_node_bt_node',
    'nav2_pipeline_sequence_bt_node',
    'nav2_round_robin_node_bt_node',
    'nav2_transform_available_condition_bt_node',
    'nav2_time_expired_condition_bt_node',
    'nav2_path_expiring_timer_condition',
    'nav2_distance_traveled_condition_bt_node',
    'nav2_single_trigger_bt_node',
    'nav2_goal_updated_controller_bt_node',
    'nav2_is_battery_low_condition_bt_node',
    'nav2_navigate_through_poses_action_bt_node',
    'nav2_navigate_to_pose_action_bt_node',
    'nav2_remove_passed_goals_action_bt_node',
    'nav2_planner_selector_bt_node',
    'nav2_controller_selector_bt_node',
    'nav2_goal_checker_selector_bt_node',
    'nav2_controller_cancel_bt_node',
    'nav2_path_longer_on_approach_bt_node',
    'nav2_wait_cancel_bt_node',
    'nav2_spin_cancel_bt_node',
    'nav2_back_up_cancel_bt_node',
    'nav2_drive_on_heading_cancel_bt_node',
]


def _default_config_dir() -> str:
    """최상위 config 디렉토리 기본값: $ROS_WS/config, 없으면 컨테이너 규약 경로."""
    ros_ws = os.environ.get('ROS_WS', '')
    return os.path.join(ros_ws, 'config') if ros_ws else '/ros2_ws/config'


def ttc_plugin_available() -> bool:
    """amr_behavior 의 IsTTCBelowThreshold BT 플러그인 라이브러리가 설치돼 있는가."""
    try:
        prefix = get_package_prefix('amr_behavior')
    except PackageNotFoundError:
        return False
    return os.path.isfile(os.path.join(prefix, 'lib', f'lib{TTC_BT_LIB}.so'))


def _setup(context):
    share = get_package_share_directory(PKG)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    config_dir = LaunchConfiguration('config_dir').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    respawn = LaunchConfiguration('use_respawn').perform(context).lower() == 'true'
    ttc_mode = LaunchConfiguration('use_ttc_bt').perform(context).lower()
    profiler_file = LaunchConfiguration('profiler_params_file').perform(context)

    use_ttc = ttc_plugin_available() if ttc_mode == 'auto' else ttc_mode == 'true'
    suffix = '' if use_ttc else '_no_ttc'
    bt_to_pose = os.path.join(share, 'behavior_trees', f'navigate_to_pose{suffix}.xml')
    bt_through = os.path.join(share, 'behavior_trees', f'navigate_through_poses{suffix}.xml')
    bt_libs = NAV2_BT_LIBS + ([TTC_BT_LIB] if use_ttc else [])

    robot_ns = '/' + robot_name.strip('/') if robot_name.strip('/') else ''
    templated = ReplaceString(source_file=params_file,
                              replacements={'<prefix>': prefix, '<robot_ns>': robot_ns})
    configured = ParameterFile(
        RewrittenYaml(
            source_file=templated, root_key=robot_name,
            # RewrittenYaml 은 YAML 에 이미 있는 키만 바꾼다 → BT 경로·플러그인 목록은 bt_navigator 에 직접 넘긴다
            param_rewrites={'use_sim_time': use_sim_time, 'autostart': autostart},
            convert_types=True),
        allow_substs=True)
    robot_params = os.path.join(config_dir, 'robot_params.yaml')
    extra = [robot_params] if os.path.isfile(robot_params) else []
    sim_time = use_sim_time.lower() == 'true'
    ros_args = ['--ros-args', '--log-level', log_level]
    common = dict(namespace=robot_name, output='screen', respawn=respawn, respawn_delay=2.0,
                  arguments=ros_args)

    actions = [
        LogInfo(msg=f'[navigation] ns={robot_name or "/"} prefix="{prefix}" BT={bt_to_pose} '
                    f'robot_params={robot_params if extra else "(없음: 플러그인 기본 한계 사용)"}'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[configured], **common),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[configured] + extra, remappings=[('cmd_vel', 'cmd_vel_nav')], **common),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[configured], remappings=[('cmd_vel', 'cmd_vel_nav')], **common),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[configured, {'plugin_lib_names': bt_libs,
                                      'default_nav_to_pose_bt_xml': bt_to_pose,
                                      'default_nav_through_poses_bt_xml': bt_through}],
             **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation',
             parameters=[{'use_sim_time': sim_time, 'autostart': autostart.lower() == 'true',
                          'node_names': LIFECYCLE_NODES, 'bond_timeout': 4.0}],
             namespace=robot_name, output='screen', arguments=ros_args),
        Node(package=PKG, executable='velocity_profiler_node', name='velocity_profiler_node',
             parameters=extra + [profiler_file, {'use_sim_time': sim_time}], **common),
    ]
    return actions


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory(PKG)
    args = [
        DeclareLaunchArgument('robot_name', default_value='amr_01',
                              description='로봇 네임스페이스 (빈 문자열이면 네임스페이스 없음)'),
        DeclareLaunchArgument('prefix', default_value='',
                              description="TF 프레임 접두어 (다중 로봇: '<robot_name>/')"),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('profiler_params_file',
                              default_value=os.path.join(share, 'config',
                                                         'velocity_profiler.yaml')),
        DeclareLaunchArgument('config_dir', default_value=_default_config_dir(),
                              description='robot_params.yaml 디렉토리 (limits.* 단일 출처)'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('use_ttc_bt', default_value='auto',
                              description='auto: amr_behavior TTC 플러그인이 있으면 사용'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
