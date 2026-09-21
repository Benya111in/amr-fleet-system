"""
fleet_adapter_node — 로봇별 상태 취합기 (components.md §3.6 / §5.6). 로봇 네임스페이스에서 뜬다.

- Sub  odometry/filtered_map (Odometry), battery_state (BatteryState), task_status (Task),
       executor/phase (String), safety/estop_active (Bool), safety/zone (String)
- Pub  robot_state (RobotState, 2 Hz; 송신 지연 큐 U(comm_latency_ms) ms, drop_rate 유실, 링크 FIFO)
- SrvS assign_task (AssignTask; serve_assign_task=true 일 때): IDLE 이면 수락하고 현재 작업으로 저장.
       통합 시스템에서 task_executor_node 가 이 서비스를 직접 제공하면 serve_assign_task=false 로 끈다.
- auto_complete_after_s > 0: 실행기 없이 IN_PROGRESS → (N s) → COMPLETED 를 task_status 로 흉내 낸다.
- 로그 logs/comm_latency_<robot>_YYYYmmdd.csv [cmd_time, response_time, latency_ms] (명세 4.10 포맷):
       cmd_time = fleet 이 찍은 요청 stamp, response_time = assign_task 수신 (multi_robot.md §6 측정).
       명령 → 첫 움직임 응답 시간은 amr_evaluation 의 response_time_logger 가 잰다.
"""

from __future__ import annotations

import csv
import datetime as _dt
import os
import pathlib
from typing import Optional
import zlib

from amr_fleet.kpi import STATUS_ERROR, STATUS_ESTOP
from amr_fleet.latency import LatencyModel
from amr_fleet.robot_status import (
    STATUS_NAMES, battery_percent, map_status,
)
from amr_fleet.task_msg import time_to_float
from amr_fleet.task_schema import STATUS_COMPLETED, STATUS_FAILED, STATUS_IN_PROGRESS
from amr_fleet.timer_queue import TimerQueue
from amr_msgs.msg import RobotState, Task
from amr_msgs.srv import AssignTask
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

LATENCY_LOG_HEADER = ['cmd_time', 'response_time', 'latency_ms']


class FleetAdapterNode(Node):
    """로봇 상태 취합 → robot_state 2 Hz, assign_task 수락, 지연 큐."""

    def __init__(self, **kwargs):
        super().__init__('fleet_adapter_node', **kwargs)
        defaults = {
            'robot_id': '',                  # '' = 네임스페이스에서 유도 (/amr_01 → amr_01)
            'publish_rate_hz': 2.0,
            'simulate_latency': True,        # false = robot_state 를 지연 없이 발행 (실기)
            'comm_latency_ms': [0.0, 100.0],  # robot_state 메시지마다 U(lo, hi) ms (숫자면 [0, 값])
            'drop_rate': 0.0,                # robot_state 유실 확률
            'seed': 0,                       # 0 = 무작위. 0 이 아니면 robot_id 별로 달리 섞는다
            'auto_complete_after_s': 0.0,    # > 0 이면 수락 N s 뒤 COMPLETED 를 흉내 (테스트용)
            'serve_assign_task': True,
            'initial_battery': 100.0,        # battery_state 가 없을 때 보고할 잔량 [%]
            'log_dir': '',                   # '' = $ROS_WS/logs
        }
        for name, value in defaults.items():
            descriptor = ParameterDescriptor(dynamic_typing=name == 'comm_latency_ms')
            self.declare_parameter(name, value, descriptor)
        p = self._param
        self.robot_id = p('robot_id') or self.get_namespace().strip('/').split('/')[-1] or 'amr'
        seed = int(p('seed'))
        if seed:
            seed += zlib.crc32(self.robot_id.encode())   # 로봇마다 다른 재현 가능한 시퀀스
        self._latency = LatencyModel(simulate=bool(p('simulate_latency')),
                                     comm_latency_ms=p('comm_latency_ms'),
                                     drop_rate=float(p('drop_rate')), seed=seed)
        self._auto_complete_s = float(p('auto_complete_after_s'))
        default_log_dir = os.path.join(os.environ.get('ROS_WS', '.'), 'logs')
        self._log_dir = pathlib.Path(p('log_dir') or default_log_dir)

        # 최근 상태
        self._odom: Optional[Odometry] = None
        self._battery = float(p('initial_battery'))
        self._phase = ''
        self._estop = False
        self._zone = ''
        self._current: Optional[Task] = None
        self._state_queue = TimerQueue(self, self._pub_state_now)       # robot_state 송신 지연
        self._auto_queue = TimerQueue(self, self._auto_complete)         # 모의 완료 (task_id)

        # ROS 인터페이스 (상대 이름)
        self._pub_state = self.create_publisher(RobotState, 'robot_state', 10)
        # task_status 는 task_executor_node 가 발행한다. 모의 완료(테스트) 때만 어댑터가 대신 낸다
        self._pub_task_status = (self.create_publisher(Task, 'task_status', 10)
                                 if self._auto_complete_s > 0.0 else None)
        self.create_subscription(Odometry, 'odometry/filtered_map', self._on_odom, 10)
        self.create_subscription(BatteryState, 'battery_state', self._on_battery, 10)
        self.create_subscription(Task, 'task_status', self._on_task_status, 10)
        self.create_subscription(String, 'executor/phase', self._on_phase, 10)
        self.create_subscription(Bool, 'safety/estop_active', self._on_estop, 10)
        self.create_subscription(String, 'safety/zone', self._on_zone, 10)
        if bool(p('serve_assign_task')):
            self.create_service(AssignTask, 'assign_task', self._on_assign_task)

        self.create_timer(1.0 / max(float(p('publish_rate_hz')), 0.1), self._on_publish_timer)
        self.get_logger().info(
            f'fleet_adapter {self.robot_id}: rate={p("publish_rate_hz")} Hz, '
            f'latency={self._latency.describe()}, auto_complete={self._auto_complete_s} s, '
            f'serve_assign_task={p("serve_assign_task")}')

    def _param(self, name: str):
        return self.get_parameter(name).value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ 입력
    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery = battery_percent(msg.percentage, fallback=self._battery)

    def _on_phase(self, msg: String) -> None:
        self._phase = msg.data

    def _on_estop(self, msg: Bool) -> None:
        if msg.data != self._estop:
            self.get_logger().warn(f'{self.robot_id}: estop {"활성" if msg.data else "해제"}')
        self._estop = bool(msg.data)

    def _on_zone(self, msg: String) -> None:
        self._zone = msg.data

    def _on_task_status(self, msg: Task) -> None:
        if msg.status == STATUS_IN_PROGRESS:
            if self._current is None or self._current.task_id != msg.task_id:
                self._current = msg
        elif msg.status in (STATUS_COMPLETED, STATUS_FAILED):
            if self._current is not None and self._current.task_id == msg.task_id:
                self._current = None      # 실행기(또는 외부)가 끝냄 → 대기 중인 모의 완료는 무시
                if self._auto_complete_s > 0.0:
                    self._phase = 'idle'

    # ------------------------------------------------------------------ assign_task
    def _on_assign_task(self, req: AssignTask.Request, resp: AssignTask.Response):
        now = self._now()
        cmd_time = time_to_float(req.task.header.stamp)
        if cmd_time is not None:
            self._append_csv(f'comm_latency_{self.robot_id}', LATENCY_LOG_HEADER,
                             [f'{cmd_time:.6f}', f'{now:.6f}', f'{(now - cmd_time) * 1e3:.3f}'])
        resp.robot_id = self.robot_id
        status = map_status(self._estop, self._phase)
        if status in (STATUS_ESTOP, STATUS_ERROR):
            resp.success, resp.message = False, STATUS_NAMES[status].lower()
            return resp
        if self._current is not None:
            resp.success, resp.message = False, f'busy:{self._current.task_id}'
            return resp

        task = req.task
        task.robot_id = self.robot_id
        task.status = STATUS_IN_PROGRESS
        self._current = task
        resp.success, resp.message = True, 'accepted'
        self.get_logger().info(f'{self.robot_id}: 작업 수락 {task.task_id}'
                               + (f' (통신 지연 {(now - cmd_time) * 1e3:.1f} ms)'
                                  if cmd_time is not None else ''))
        if self._auto_complete_s > 0.0:
            self._phase = 'moving'
            self._publish_task_status(task, STATUS_IN_PROGRESS)
            self._auto_queue.push(task.task_id, self._auto_complete_s)
        return resp

    def _auto_complete(self, task_id: str) -> None:
        task = self._current
        if task is None or task.task_id != task_id:
            return          # 그 사이 실패/취소된 작업
        self._current = None
        self._phase = 'idle'
        self._publish_task_status(task, STATUS_COMPLETED)
        self.get_logger().info(f'{self.robot_id}: 작업 완료(모의) {task.task_id}')

    def _publish_task_status(self, task: Task, status: int) -> None:
        msg = Task()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = task.header.frame_id
        msg.task_id, msg.robot_id, msg.priority = task.task_id, self.robot_id, task.priority
        msg.deadline, msg.pickup_pose, msg.dropoff_pose = task.deadline, task.pickup_pose, \
            task.dropoff_pose
        msg.item_type, msg.item_mass, msg.status = task.item_type, task.item_mass, status
        self._pub_task_status.publish(msg)

    # ------------------------------------------------------------------ robot_state
    def _build_state(self) -> RobotState:
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.robot_id = self.robot_id
        if self._odom is not None:
            msg.pose.header = self._odom.header
            msg.pose.pose = self._odom.pose.pose
        else:
            msg.pose.header.frame_id = 'map'
        msg.battery_level = float(self._battery)
        msg.current_task_id = self._current.task_id if self._current is not None else ''
        msg.status = map_status(self._estop, self._phase)
        return msg

    def _on_publish_timer(self) -> None:
        msg = self._build_state()
        if not self._latency.enabled:
            self._pub_state.publish(msg)
            return
        delay = self._latency.sample()
        if delay is not None:           # None = 유실(drop_rate)
            self._state_queue.push(msg, delay, key='robot_state')   # 추월 없이 (링크 FIFO)

    def _pub_state_now(self, msg: RobotState) -> None:
        self._pub_state.publish(msg)

    # ------------------------------------------------------------------ 로그
    def _append_csv(self, prefix: str, header, row) -> None:
        try:
            self._log_dir.mkdir(parents=True, exist_ok=True)
            day = _dt.datetime.fromtimestamp(self._now()).strftime('%Y%m%d')
            path = self._log_dir / f'{prefix}_{day}.csv'
            new_file = not path.exists() or path.stat().st_size == 0
            with open(path, 'a', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                if new_file:
                    w.writerow(header)
                w.writerow(row)
        except OSError as exc:
            self.get_logger().warn(f'로그 쓰기 실패 {prefix}: {exc}')


def main(args=None):
    """ros2 run amr_fleet fleet_adapter_node."""
    rclpy.init(args=args)
    node = FleetAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
