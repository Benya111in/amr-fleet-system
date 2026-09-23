"""
kidnap_monitor_node: 위치 상실(납치) 감지 → 전역 재초기화 + 제자리 회전 → 수렴 확인 (명세 4.3).

인터페이스 (docs/architecture/components.md §5.2)
  Sub  amcl_pose                 geometry_msgs/PoseWithCovarianceStamped  공분산 급증 / 점프
  Sub  odometry/filtered         nav_msgs/Odometry   odom 프레임 연속 자세 (점프 판정 + AMCL 사이 보간)
  Sub  scan_filtered             sensor_msgs/LaserScan
  Sub  /map                      nav_msgs/OccupancyGrid (transient_local)
  Pub  localization/lost         std_msgs/Bool (latched) — BT IsLocalized 가 구독
  Pub  localization/kidnap_status std_msgs/String (JSON, 1 Hz + 전이 시) — 디버그/대시보드
  Pub  initialpose               geometry_msgs/PoseWithCovarianceStamped — 가설 시드 (AMCL 구독)
  SrvC reinitialize_global_localization  std_srvs/Empty (AMCL)
  SrvC request_nomotion_update   std_srvs/Empty (AMCL) — 정지 직후 nomotion_updates_on_stop 회
  ActC spin                      nav2_msgs/action/Spin (behavior_server)
  SrvC <ekf_set_pose_service>    robot_localization/srv/SetPose (선택, 복구 즉시 map EKF 이동)
판정 로직은 amr_localization.kidnap_detector, 전역 가설 탐색은 amr_localization.global_seed
(둘 다 rclpy 비의존, pytest 대상). 재초기화 시도 k ≤ seed_attempts 는 k 번째 가설을 initialpose 로,
그 뒤는 AMCL 균일 재초기화(reinitialize_global_localization)로 한다. 복구 중에는 현재 추정이 아닌 가설(별칭)을
탐색 이후 odom 이동만큼 옮겨 같은 스캔으로 인라이어 비율을 재고, 현재 추정이 converge_margin 이상 앞설 때만
수렴으로 본다 (별칭 거부, docs/algorithms/slam.md §5.2).
spin 서버가 없으면(단독 시험) fallback_cmd_vel_topic 이 설정된 경우에만 직접 회전 명령을 낸다 —
운용 스택에서는 cmd_vel 발행자가 safety_node 하나여야 하므로 기본값은 비활성('').
"""

import collections
import csv
import json
import math
import os
from pathlib import Path
import time
from typing import Deque, List, Optional

from action_msgs.msg import GoalStatus
from amr_localization import global_seed
from amr_localization import kidnap_detector as kd
from amr_localization.scan_map_match import DistanceField, GridSpec, match_ratio
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import Spin
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty


def yaw_of(q) -> float:
    """쿼터니언 → yaw."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_sec(stamp) -> float:
    """builtin_interfaces/Time → 초."""
    return stamp.sec + stamp.nanosec * 1e-9


class KidnapMonitorNode(Node):
    """납치 감지·복구 노드."""

    def __init__(self, **kwargs) -> None:
        super().__init__('kidnap_monitor_node', **kwargs)
        defaults = kd.KidnapParams()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        self.declare_parameter('amcl_pose_topic', 'amcl_pose')
        self.declare_parameter('odom_topic', 'odometry/filtered')
        self.declare_parameter('initial_pose_topic', 'initialpose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('seed_attempts', 3)                   # 0 = 시드 없이 AMCL 전역만
        self.declare_parameter('seed_cov_xy', 0.0625)                # [m²] σ 0.25 m
        self.declare_parameter('seed_var_yaw', 0.01)                 # [rad²] (σ ≈ 5.7°)
        self.declare_parameter('scan_topic', 'scan_filtered')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('lost_topic', 'localization/lost')
        # 도킹 서버가 다른 도크의 마커를 보고 역산한 로봇 자세 (amr_behavior docking_server_node)
        self.declare_parameter('marker_fix_topic', 'localization/marker_fix')
        self.declare_parameter('status_topic', 'localization/kidnap_status')
        self.declare_parameter('reinit_service', 'reinitialize_global_localization')
        self.declare_parameter('spin_action', 'spin')
        self.declare_parameter('spin_time_allowance', 30.0)          # [s]
        self.declare_parameter('ekf_set_pose_service', '')           # '' = 사용 안 함
        self.declare_parameter('fallback_cmd_vel_topic', '')         # '' = 직접 회전 안 함
        self.declare_parameter('fallback_angular_speed', 0.5)        # [rad/s]
        self.declare_parameter('lidar_offset', [0.15, 0.0, 0.0])     # [m, m, rad] base→lidar
        self.declare_parameter('inlier_dist', 0.2)                   # [m]
        self.declare_parameter('max_beams', 180)
        self.declare_parameter('occupied_thresh', 65)
        self.declare_parameter('match_rate', 2.0)                    # [Hz]
        self.declare_parameter('tick_rate', 5.0)                     # [Hz]
        self.declare_parameter('event_log', '')                      # CSV 경로 ('' = 기록 안 함)
        self.declare_parameter('nomotion_updates_on_stop', 3)        # 정지당 AMCL 무이동 갱신 수
        self.declare_parameter('nomotion_period', 1.0)               # [s]
        self.declare_parameter('nomotion_service', 'request_nomotion_update')

        params = kd.KidnapParams(**{
            name: type(value)(self.get_parameter(name).value)
            for name, value in vars(defaults).items()})
        self.detector = kd.KidnapDetector(params)
        self.lidar_offset = tuple(float(v) for v in self.get_parameter('lidar_offset').value)
        self.inlier_dist = float(self.get_parameter('inlier_dist').value)
        self.max_beams = int(self.get_parameter('max_beams').value)
        self.occupied_thresh = int(self.get_parameter('occupied_thresh').value)
        self.spin_allowance = float(self.get_parameter('spin_time_allowance').value)
        self.fallback_speed = float(self.get_parameter('fallback_angular_speed').value)

        self.field: Optional[DistanceField] = None
        self.scans: Deque[LaserScan] = collections.deque(maxlen=20)
        self.last_scan: Optional[LaserScan] = None
        self.seeds: List[global_seed.Hypothesis] = []
        self.seed_odom_yaw = 0.0
        self.seed_odom_pose: Optional[kd.Pose] = None     # 가설 탐색 시각의 odom 자세
        self.last_odom_yaw = 0.0
        self.last_alias_margin: Optional[float] = None
        self.seed_attempts = int(self.get_parameter('seed_attempts').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.last_ratio: Optional[float] = None
        self.last_amcl_cov = (0.0, 0.0)
        self.spin_goal = None
        self.fallback_until: Optional[float] = None
        self.fallback_sign = 1.0
        self.events_written = 0

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.lost_pub = self.create_publisher(Bool, self.get_parameter('lost_topic').value,
                                              latched)
        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        self.create_subscription(PoseWithCovarianceStamped,
                                 self.get_parameter('amcl_pose_topic').value, self.on_amcl, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self.on_odom, 20)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value,
                                 self.on_map, latched)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter('initial_pose_topic').value, 10)
        self.reinit_client = self.create_client(Empty,
                                                self.get_parameter('reinit_service').value)
        self.stop_scheduler = kd.StopUpdateScheduler(
            count=int(self.get_parameter('nomotion_updates_on_stop').value),
            period=float(self.get_parameter('nomotion_period').value))
        self.nomotion_client = self.create_client(
            Empty, self.get_parameter('nomotion_service').value)
        self.nomotion_calls = 0
        self.spin_client = ActionClient(self, Spin, self.get_parameter('spin_action').value)
        self.set_pose_client = None
        set_pose_srv = self.get_parameter('ekf_set_pose_service').value
        if set_pose_srv:
            try:
                from robot_localization.srv import SetPose
                self.set_pose_client = self.create_client(SetPose, set_pose_srv)
                self.SetPose = SetPose
            except ImportError:  # pragma: no cover - 설치 환경 문제
                self.get_logger().warn('robot_localization.srv 없음: EKF set_pose 비활성')
        fallback = self.get_parameter('fallback_cmd_vel_topic').value
        self.cmd_pub = self.create_publisher(Twist, fallback, 10) if fallback else None
        self.create_subscription(
            PoseWithCovarianceStamped, self.get_parameter('marker_fix_topic').value,
            self.on_marker_fix, 10)

        self.event_log = self.get_parameter('event_log').value
        self.create_timer(1.0 / max(float(self.get_parameter('match_rate').value), 0.1),
                          self.on_match_timer)
        self.create_timer(1.0 / max(float(self.get_parameter('tick_rate').value), 0.1),
                          self.on_tick)
        self.create_timer(1.0, self.publish_status)
        self.publish_lost(False)
        self.get_logger().info(
            f'kidnap monitor: cov>{params.cov_thresh} m^2, jump>{params.jump_thresh} m, '
            f'inlier<{params.match_thresh} x{params.match_window} scans '
            f'(d<{self.inlier_dist} m), spin {math.degrees(params.spin_angle):.0f} deg')

    # --------------------------------------------------------------- 시각
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --------------------------------------------------------------- 콜백
    def on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        spec = GridSpec(info.width, info.height, info.resolution,
                        info.origin.position.x, info.origin.position.y,
                        yaw_of(info.origin.orientation))
        t0 = time.monotonic()
        self.field = DistanceField(msg.data, spec, self.occupied_thresh)
        self.get_logger().info(
            f'map {info.width}x{info.height} @ {info.resolution:.3f} m: distance field built in '
            f'{(time.monotonic() - t0) * 1e3:.0f} ms')

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.last_odom_yaw = yaw_of(p.orientation)
        t = stamp_sec(msg.header.stamp)
        self.detector.on_odom(t, (p.position.x, p.position.y, self.last_odom_yaw))
        tw = msg.twist.twist
        if (self.stop_scheduler.update(t, tw.linear.x, tw.angular.z)
                and self.detector.state == kd.State.TRACKING
                and self.nomotion_client.service_is_ready()):
            # 정지 직후 마지막 AMCL 갱신 오차가 굳지 않게 같은 자세의 스캔으로 한 번 더 갱신
            self.nomotion_client.call_async(Empty.Request())
            self.nomotion_calls += 1

    def on_scan(self, msg: LaserScan) -> None:
        self.scans.append(msg)
        self.last_scan = msg

    def on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        c = msg.pose.covariance
        self.last_amcl_cov = (c[0] + c[7], c[35])
        self.execute(self.detector.on_amcl(
            stamp_sec(msg.header.stamp), (p.position.x, p.position.y, yaw_of(p.orientation)),
            c[0] + c[7], c[35]))

    def on_marker_fix(self, msg: PoseWithCovarianceStamped) -> None:
        """등록된 마커로 역산한 자세: 현재 추정과 크게 다르면 LOST 선언 + 그 자세로 시드."""
        p = msg.pose.pose
        self.execute(self.detector.on_marker_fix(
            self.now_sec(), (p.position.x, p.position.y, yaw_of(p.orientation))))

    def on_match_timer(self) -> None:
        """가장 최근 스캔의 일치도: 스캔 시각 map 자세 = 마지막 AMCL ∘ odom 상대 이동."""
        if self.field is None or not self.scans:
            return
        scan = self.scans.pop()
        self.scans.clear()
        t_scan = stamp_sec(scan.header.stamp)
        pose = self.detector.map_pose_at(t_scan)
        if pose is None:
            return
        ratio, valid = match_ratio(self.field, pose, self.lidar_offset, scan.ranges,
                                   scan.angle_min, scan.angle_increment, scan.range_max,
                                   self.inlier_dist, self.max_beams)
        self.last_ratio = ratio
        self.last_alias_margin = self.alias_margin(scan, t_scan, pose)
        self.execute(self.detector.on_match(t_scan, ratio, valid, self.last_alias_margin))

    def alias_margin(self, scan: LaserScan, t_scan: float, pose: kd.Pose) -> Optional[float]:
        """복구 중: ρ(현재 추정) − max ρ(현재 추정이 아닌 전역 가설 = 별칭), 둘 다 국소 정밀화."""
        if (self.detector.state not in (kd.State.RECOVERING, kd.State.FAILED) or not self.seeds
                or self.seed_odom_pose is None):
            return None
        odom_now = self.detector.odom_pose_at(t_scan)
        if odom_now is None:
            return None
        motion = kd.relative_pose(self.seed_odom_pose, odom_now)
        alternatives = global_seed.alternative_poses(self.seeds, pose, motion)
        beams = global_seed.base_beams(scan.ranges, scan.angle_min, scan.angle_increment,
                                       scan.range_max, self.max_beams, self.lidar_offset)
        return global_seed.alias_margin(self.field, beams, pose, alternatives, self.inlier_dist)

    def on_tick(self) -> None:
        now = self.now_sec()
        if self.fallback_until is not None:
            self.fallback_step(now)
        self.execute(self.detector.tick(now))

    # --------------------------------------------------------------- 동작
    def execute(self, actions: List[kd.Action]) -> None:
        """상태기 동작을 ROS 호출로."""
        for action in actions:
            if action.kind == kd.ActionType.PUBLISH_LOST:
                self.publish_lost(bool(action.value))
            elif action.kind == kd.ActionType.REINITIALIZE:
                self.reinitialize(int(action.value or 1))
            elif action.kind == kd.ActionType.SPIN:
                self.start_spin(float(action.value))
            elif action.kind == kd.ActionType.CANCEL_SPIN:
                self.cancel_spin()
            elif action.kind == kd.ActionType.SET_EKF_POSE:
                self.set_ekf_pose(*action.value)
            elif action.kind == kd.ActionType.SEED_POSE:
                self.seed_pose(action.value)
        if actions:
            self.log_events()
            self.publish_status()

    def publish_lost(self, lost: bool) -> None:
        self.lost_pub.publish(Bool(data=lost))
        # rclpy 로거는 호출 위치마다 심각도가 고정이라 분기마다 따로 부른다
        if lost:
            self.get_logger().warn('localization/lost = True')
        else:
            self.get_logger().info('localization/lost = False')

    def reinitialize(self, attempt: int) -> None:
        """시도 k: k ≤ seed_attempts 면 k 번째 가설 시드, 아니면 AMCL 균일 재초기화."""
        if attempt <= self.seed_attempts and self.seed_initial_pose(attempt):
            return
        if not self.reinit_client.service_is_ready():
            self.get_logger().error('reinitialize_global_localization 서비스가 없다 (AMCL 미기동?)')
            return
        self.reinit_client.call_async(Empty.Request())
        self.get_logger().warn('AMCL global re-initialization requested')

    def seed_initial_pose(self, attempt: int) -> bool:
        """가설 탐색 (첫 시도에 1 회) 후 attempt 번째 가설을 initialpose 로. 불가하면 False."""
        if attempt == 1 or not self.seeds:
            if self.field is None or self.last_scan is None:
                return False
            scan = self.last_scan
            t0 = time.monotonic()
            self.seeds = global_seed.search(
                self.field, self.field.free, scan.ranges, scan.angle_min, scan.angle_increment,
                scan.range_max, self.lidar_offset)
            self.seed_odom_yaw = self.last_odom_yaw
            self.seed_odom_pose = self.detector.odom_pose_at(stamp_sec(scan.header.stamp))
            self.get_logger().info(
                f'global seed search: {len(self.seeds)} hypotheses in '
                f'{time.monotonic() - t0:.2f} s: ' + ', '.join(
                    f'({h.x:.2f}, {h.y:.2f}, {math.degrees(h.yaw):.0f} deg, '
                    f'{h.score:.3f} m, inlier {h.ratio:.3f})'
                    for h in self.seeds))
        if attempt > len(self.seeds):
            return False
        h = self.seeds[attempt - 1]
        # 탐색 이후 제자리 회전한 만큼 헤딩을 옮긴다 (odom 헤딩 변화, 위치는 그대로)
        yaw = h.yaw + kd.wrap_angle(self.last_odom_yaw - self.seed_odom_yaw)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = h.x
        msg.pose.pose.position.y = h.y
        msg.pose.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.pose.orientation.w = math.cos(0.5 * yaw)
        cov = [0.0] * 36
        cov[0] = cov[7] = float(self.get_parameter('seed_cov_xy').value)
        cov[35] = float(self.get_parameter('seed_var_yaw').value)
        msg.pose.covariance = cov
        self.initial_pose_pub.publish(msg)
        self.get_logger().warn(
            f'seed #{attempt}: initialpose ({h.x:.2f}, {h.y:.2f}, {math.degrees(yaw):.0f} deg), '
            f'score {h.score:.3f} m, inlier {h.ratio:.3f}')
        return True

    def seed_pose(self, pose) -> None:
        """외부 증거(마커)로 역산한 자세를 AMCL·EKF 에 그대로 넣는다 (전역 탐색 없이)."""
        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.pose.orientation.w = math.cos(0.5 * yaw)
        cov = [0.0] * 36
        cov[0] = cov[7] = float(self.get_parameter('seed_cov_xy').value)
        cov[35] = float(self.get_parameter('seed_var_yaw').value)
        msg.pose.covariance = cov
        self.initial_pose_pub.publish(msg)
        self.set_ekf_pose((float(x), float(y), float(yaw)),
                          float(self.get_parameter('seed_cov_xy').value),
                          float(self.get_parameter('seed_var_yaw').value))
        self.get_logger().warn(
            f'marker fix seed: initialpose ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)')

    def start_spin(self, angle: float) -> None:
        if self.spin_client.server_is_ready():
            goal = Spin.Goal()
            goal.target_yaw = float(angle)
            goal.time_allowance = Duration(sec=int(self.spin_allowance),
                                           nanosec=int((self.spin_allowance % 1.0) * 1e9))
            future = self.spin_client.send_goal_async(goal)
            future.add_done_callback(self.on_spin_accepted)
            self.get_logger().info(f'spin {math.degrees(angle):.0f} deg requested')
        elif self.cmd_pub is not None:
            self.fallback_sign = 1.0 if angle >= 0.0 else -1.0
            self.fallback_until = self.now_sec() + abs(angle) / max(self.fallback_speed, 1e-3)
            self.get_logger().info(
                f'spin server unavailable: rotating via {self.cmd_pub.topic_name} '
                f'at {self.fallback_speed:.2f} rad/s')
        else:
            self.get_logger().error('spin 액션 서버도 fallback cmd_vel 도 없다: 회전 없이 대기')
            self.execute(self.detector.on_spin_done(self.now_sec(), False))

    def on_spin_accepted(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('spin goal rejected')
            self.execute(self.detector.on_spin_done(self.now_sec(), False))
            return
        self.spin_goal = handle
        handle.get_result_async().add_done_callback(self.on_spin_result)

    def on_spin_result(self, future) -> None:
        self.spin_goal = None
        result = future.result()
        ok = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
        if result is not None and result.status == GoalStatus.STATUS_CANCELED:
            return  # 상태기가 스스로 취소한 경우
        self.execute(self.detector.on_spin_done(self.now_sec(), ok))

    def fallback_step(self, now: float) -> None:
        twist = Twist()
        if now < self.fallback_until:
            twist.angular.z = self.fallback_sign * self.fallback_speed
            self.cmd_pub.publish(twist)
            return
        self.cmd_pub.publish(twist)
        self.fallback_until = None
        self.execute(self.detector.on_spin_done(now, True))

    def cancel_spin(self) -> None:
        if self.spin_goal is not None:
            self.spin_goal.cancel_goal_async()
            self.spin_goal = None
        if self.fallback_until is not None and self.cmd_pub is not None:
            self.cmd_pub.publish(Twist())
            self.fallback_until = None

    def set_ekf_pose(self, pose: kd.Pose, cov_xy: float, var_yaw: float) -> None:
        if self.set_pose_client is None:
            return
        if not self.set_pose_client.service_is_ready():
            self.get_logger().warn('EKF set_pose 서비스 없음: EKF 가 AMCL 을 따라가도록 둔다')
            return
        req = self.SetPose.Request()
        msg = req.pose
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        msg.pose.pose.orientation.z = math.sin(0.5 * pose[2])
        msg.pose.pose.orientation.w = math.cos(0.5 * pose[2])
        cov = [0.0] * 36
        cov[0] = cov[7] = max(cov_xy / 2.0, 1e-4)
        cov[35] = max(var_yaw, 1e-4)
        msg.pose.covariance = cov
        self.set_pose_client.call_async(req)
        self.get_logger().info(
            f'map EKF set_pose -> ({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.0f} deg)')

    # --------------------------------------------------------------- 보고
    def status_dict(self) -> dict:
        d = self.detector
        return {
            'state': d.state.value, 'lost': d.lost,
            'match_ratio': None if self.last_ratio is None else round(self.last_ratio, 3),
            'alias_margin': (None if self.last_alias_margin is None
                             else round(self.last_alias_margin, 3)),
            'alias_rejections': d.alias_rejections,
            'amcl_cov_xy': round(self.last_amcl_cov[0], 4),
            'amcl_var_yaw': round(self.last_amcl_cov[1], 4),
            'recoveries': len(d.recovery_times),
            'last_recovery_s': round(d.recovery_times[-1], 2) if d.recovery_times else None,
            'nomotion_updates': self.nomotion_calls,
        }

    def publish_status(self) -> None:
        self.status_pub.publish(String(data=json.dumps(self.status_dict())))

    def log_events(self) -> None:
        """새 이벤트를 로그와 (설정 시) CSV 로."""
        new = self.detector.events[self.events_written:]
        for e in new:
            self.get_logger().info(f'[{e.state}] {e.reason}')
        if self.event_log and new:
            path = Path(os.path.expandvars(self.event_log))
            path.parent.mkdir(parents=True, exist_ok=True)
            first = not path.exists()
            with path.open('a', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                if first:
                    w.writerow(['t', 'state', 'reason'])
                for e in new:
                    w.writerow([f'{e.t:.3f}', e.state, e.reason])
        self.events_written = len(self.detector.events)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KidnapMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
