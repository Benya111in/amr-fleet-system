"""
E-stop 조작 결과 (순수 파이썬, rclpy 미의존).

dashboard_node.publish_estop 이 채우고 web_app 이 기다렸다가 HTTP 응답·표시 상태에 반영한다.
해제는 값(false) 발행 + 로봇별 safety/reset_estop(Trigger) 호출이며, safety_node 의 래치는 서비스로만
풀리므로(sequences.md, components.md §5.4) 리셋이 성공한 로봇만 "해제됨" 으로 본다.

로봇별 리셋 상태:
    ok              success=true
    rejected        success=false (원인이 남아 있음 등, message 에 사유)
    no_server       서비스 서버 없음 (safety_node 미기동) — require_ack=False 면 해제로 친다
    error           호출 예외
    timeout         기다리는 동안 응답 없음
    pending         아직 응답 없음 (wait 전)
    not_configured  reset_estop_service 파라미터가 '' — 값 발행만 하는 구성
"""

import threading
import time
from typing import Dict, List, Optional

OK = 'ok'
REJECTED = 'rejected'
NO_SERVER = 'no_server'
ERROR = 'error'
TIMEOUT = 'timeout'
PENDING = 'pending'
NOT_CONFIGURED = 'not_configured'

RELEASED_STATES = (OK, NOT_CONFIGURED)


class EstopOutcome:
    """E-stop 한 번의 발행·리셋 결과. 리셋 응답은 실행기 스레드에서 set_reset 으로 들어온다 (스레드 안전)."""

    def __init__(self, target: str, active: bool, require_ack: bool = True):
        self.target = target
        self.active = bool(active)
        self.require_ack = require_ack
        self.published: List[str] = []            # 발행한 토픽
        self.publish_failed: Dict[str, str] = {}   # robot_id → 오류 (토픽 이름 불가 등)
        self.reset: Dict[str, str] = {}            # robot_id → 상태
        self.messages: Dict[str, str] = {}         # robot_id → 서버 메시지·오류
        self._cond = threading.Condition()

    def add_published(self, topic: str) -> None:
        self.published.append(topic)

    def fail_publish(self, robot_id: str, error: str) -> None:
        self.publish_failed[robot_id] = error

    def set_reset(self, robot_id: str, status: str, message: str = '') -> None:
        with self._cond:
            self.reset[robot_id] = status
            if message:
                self.messages[robot_id] = message
            self._cond.notify_all()

    def pending(self) -> List[str]:
        with self._cond:
            return [r for r, s in self.reset.items() if s == PENDING]

    def wait(self, timeout: float) -> bool:
        """대기 중인 리셋 응답을 timeout [s] 까지 기다린다. 남은 것은 timeout 으로 표시. 모두 왔으면 True."""
        deadline = time.monotonic() + max(timeout, 0.0)
        with self._cond:
            while any(s == PENDING for s in self.reset.values()):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._cond.wait(remaining)
            late = [r for r, s in self.reset.items() if s == PENDING]
            for rid in late:
                self.reset[rid] = TIMEOUT
            return not late

    def still_latched(self) -> List[str]:
        """해제 요청인데 래치가 풀렸다고 볼 수 없는 로봇 (리셋 실패·무응답·값 발행 실패)."""
        if self.active:
            return []
        ok = RELEASED_STATES if self.require_ack else RELEASED_STATES + (NO_SERVER,)
        with self._cond:
            bad = [r for r, s in self.reset.items() if s not in ok]
        return sorted(set(bad) | set(self.publish_failed))

    def ok(self) -> bool:
        return not self.publish_failed and not self.still_latched()

    def errors(self) -> List[str]:
        """사람이 읽는 실패 목록 (HTTP 응답·UI 표시용)."""
        out = [f'{rid}: E-stop 값 발행 실패 ({err})'
               for rid, err in sorted(self.publish_failed.items())]
        latched = set(self.still_latched()) - set(self.publish_failed)
        with self._cond:
            for rid in sorted(latched):
                status = self.reset.get(rid, '')
                msg = self.messages.get(rid, '')
                out.append(f'{rid}: reset_estop {status}' + (f' ({msg})' if msg else '')
                           + ' — 래치가 남아 정지 상태로 둔다')
        return out

    def detail(self) -> dict:
        with self._cond:
            return {
                'published': list(self.published),
                'reset': dict(self.reset),
                'messages': dict(self.messages),
                'publish_failed': dict(self.publish_failed),
                'still_latched': self.still_latched(),
            }


def simple_outcome(target: str, active: bool) -> EstopOutcome:
    """ROS 연동이 없을 때(테스트·단독 실행): 리셋 서비스가 없는 구성으로 본다."""
    out = EstopOutcome(target, active)
    if not active and target != 'all':
        out.set_reset(target, NOT_CONFIGURED)
    return out


def reset_status_from_result(result: Optional[object], exc: Optional[BaseException] = None):
    """Trigger 응답(또는 예외) → (상태, 메시지)."""
    if exc is not None:
        return ERROR, str(exc)
    if result is None:
        return ERROR, 'no response'
    return (OK if result.success else REJECTED), str(result.message)
