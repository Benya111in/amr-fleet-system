"""
lifecycle 감시: 전이 응답이 유실돼 멈춘 관리 노드를 직접 active 로 올린다.

왜 필요한가 (실측, 5대 기동): Nav2 `lifecycle_manager` 는 관리 노드에 `change_state` 를 부르고 응답을 기다린다.
참가자가 150 개쯤인 5대 구성에서는 그 응답이 유실되는 일이 있다 — 노드 쪽에 rclcpp 경고
`failed to send response to /amr_03/amcl/change_state (timeout): client will not receive
response` 가 남고,
매니저는 응답을 영원히 기다린다. 노드는 전이 자체를 끝냈으므로(configure 성공) 상태만 `inactive` 로 남고
그 로봇의 위치 추정이 통째로 죽는다 (스택 기동 간격을 둔 뒤에도 5대 중 1대에서 재현).

이 감시 노드는 `grace` 초 뒤부터 `period` 마다 관리 노드의 `get_state` 를 읽어,
`unconfigured` 면 configure, `inactive` 면 activate 를 **직접** 호출한다 (매니저를 기다리지 않는다).
active 가 되면 그 노드는 더 보지 않고, 전부 active 이면 조용히 계속 떠 있는다 (재확인 없음).
정상 기동에서는 grace 안에 전부 active 이므로 아무 일도 하지 않는다.

    ros2 run amr_bringup lifecycle_watchdog --ros-args -p nodes:="['/amr_01/amcl']" -p grace:=60.0

한 번에 하나씩 동기로 부른다 (감시 대상이 5~30 개, 주기 5 s). 매니저는 계속 멈춰 있으므로 그 노드의 bond 감시는
없다 — 근본 해결은 응답 유실을 없애는 것(참가자 수 축소·rmw 설정)이고, 이 노드는 기동 실패를 막는 안전망이다.
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional, Set

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node

#: 기본값 [] 은 타입이 BYTE_ARRAY 로 추론돼 문자열 배열을 거부하므로 동적 타입으로 선언한다
_NODES_DESCRIPTOR = ParameterDescriptor(dynamic_typing=True,
                                        description='감시할 lifecycle 노드 절대 이름')

#: 상태 → active 로 가는 다음 전이 (active 면 None)
NEXT_TRANSITION: Dict[int, Optional[int]] = {
    State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
    State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE,
    State.PRIMARY_STATE_ACTIVE: None,
}


def next_transition(state_id: int) -> Optional[int]:
    """`get_state` 결과에서 active 로 가는 다음 전이 (전이 중·finalized 면 None = 기다린다)."""
    return NEXT_TRANSITION.get(state_id)


class LifecycleWatchdog(Node):
    """`nodes` 파라미터의 관리 노드들을 active 로 올린다 (모듈 설명). 스핀은 `run()` 이 직접 한다."""

    def __init__(self, **kwargs) -> None:
        super().__init__('lifecycle_watchdog', **kwargs)
        declared = self.declare_parameter('nodes', [], _NODES_DESCRIPTOR)
        self.names: List[str] = [n for n in (declared.value or []) if n]
        self.grace = float(self.declare_parameter('grace', 60.0).value)
        self.period = float(self.declare_parameter('period', 5.0).value)
        self.timeout = float(self.declare_parameter('service_timeout', 3.0).value)
        self.pending: Set[str] = set(self.names)
        self._acted: Dict[str, int] = {}

    # ---------------------------------------------------------------- 서비스 호출
    def call(self, srv_type, service: str, request):
        """동기 호출 (실패·시간 초과면 None). 이 노드는 다른 곳에서 스핀하지 않는다."""
        client = self.create_client(srv_type, service)
        try:
            if not client.wait_for_service(timeout_sec=self.timeout):
                return None
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
            return future.result() if future.done() else None
        finally:
            self.destroy_client(client)

    def check_once(self) -> None:
        """멈춘 노드마다 다음 전이를 한 번씩 요청한다."""
        for name in sorted(self.pending):
            state = self.call(GetState, f'{name}/get_state', GetState.Request())
            if state is None:
                continue                       # 아직 안 뜬 노드 — 다음 주기에 다시 본다
            if state.current_state.id == State.PRIMARY_STATE_ACTIVE:
                self.pending.discard(name)
                continue
            transition = next_transition(state.current_state.id)
            if transition is None or self._acted.get(name) == transition:
                continue                       # 전이 중이거나 같은 요청을 이미 넣었다
            self._acted[name] = transition
            self.get_logger().warn(
                f'{name} 이 {state.current_state.label} 에 멈춰 있다 '
                f'(lifecycle_manager 응답 유실 추정) → 직접 전이 {transition} 요청')
            result = self.call(ChangeState, f'{name}/change_state',
                               ChangeState.Request(transition=Transition(id=transition)))
            if result is None or not result.success:
                self._acted.pop(name, None)    # 다음 주기에 다시 시도
                self.get_logger().warn(f'{name} 전이 {transition} 실패 — 다시 시도한다')

    def run(self, sleep=time.sleep) -> None:
        """`grace` 뒤부터 `period` 마다 검사. 전부 active 가 되면 검사를 멈추고 떠 있는다."""
        if not self.names:
            self.get_logger().info('감시할 노드가 없다 (nodes 파라미터 비어 있음)')
        else:
            self.get_logger().info(
                f'lifecycle 감시: {len(self.names)} 개 노드, {self.grace:g} s 뒤부터 {self.period:g} s 마다')
            sleep(self.grace)
            while rclpy.ok() and self.pending:
                self.check_once()
                if self.pending:
                    sleep(self.period)
            if rclpy.ok():
                self.get_logger().info('감시 대상이 모두 active')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LifecycleWatchdog()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
