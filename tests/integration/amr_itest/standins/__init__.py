"""
대역(stand-in) 노드: 아직 머지되지 않은 실제 노드 자리를 계약 인터페이스 그대로 채운다.

각 대역은 docs/architecture/components.md §5 의 토픽·타입·QoS 를 따르고, 대응하는 실제 노드가
설치되면 amr_itest.stack 이 자동으로 실제 노드로 바꾼다 (ITEST_STANDINS=auto). 대역은 제품이
아니라 하네스 검증용 최소 구현이다 — 결과 JSON 의 components 에 real/standin 이 기록된다.

  kinematic_sim   Gazebo + ros_gz_bridge 대역: cmd_vel → 유니사이클(가속 한계) → ground_truth/odom,
                  joint_states, imu, scan(레이캐스트), camera_info
  wheel_odometry  amr_localization/wheel_odometry_node 대역: joint_states → wheel_odom
  imu_filter      amr_localization/imu_filter_node 대역: imu/data_raw → 정지 바이어스 보정 → imu/data
  topic_relay     scan_filter_node 대역 (Gazebo): scan → scan_filtered 그대로
  amcl_pose       amcl 대역: ground_truth/odom + 가우시안 노이즈 → amcl_pose (지연 포함)
  static_tf       amr_description(robot_state_publisher) 대역: config 의 extrinsic → /tf_static
  safety_gate     amr_perception/safety_node 대역: cmd_vel_smoothed → cmd_vel (존·E-stop 래치)
  cmd_relay       amr_navigation/velocity_profiler_node 대역: cmd_vel_nav → cmd_vel_smoothed
  task_executor   amr_behavior/task_executor_node 대역: assign_task → task_status + cmd_vel_nav
"""
