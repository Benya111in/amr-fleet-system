"""
task_executor: amr_behavior/task_executor_node 대역 (components.md §5.5 의 작업 수신 부분).

  SrvS assign_task (amr_msgs/AssignTask)  IDLE 이 아니면 success=false, message="busy"
  Pub  task_status (amr_msgs/Task)        수락 즉시 IN_PROGRESS, 주행 후 COMPLETED
       executor/phase (std_msgs/String)   idle / moving
       cmd_vel_nav (Twist, 20 Hz)         drive_speed 로 drive_time 동안 직진 후 0

실제 노드는 BT 로 navigate_to_pose 를 호출하지만, 대역은 Nav2 없이 곧바로 cmd_vel_nav 를
내보낸다 — 시나리오 13 이 "명령 → 첫 움직임" 측정 사슬(assign_task → 속도 명령 체인 → 로봇)을
Nav2 머지 전에도 검증할 수 있게 하려는 것이다.
"""

from amr_itest.standins.common import RELIABLE, run, StandinNode
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class TaskExecutor(StandinNode):
    """assign_task → 짧은 직진 주행 → 완료."""

    def __init__(self):
        super().__init__('task_executor_node')
        from amr_msgs.msg import Task
        from amr_msgs.srv import AssignTask
        self.Task = Task
        ns = self.get_namespace().strip('/')
        self.robot_id = str(self.param('robot_id', '')) or ns.split('/')[-1]
        self.speed = self.param('drive_speed', 0.3)
        self.drive_time = self.param('drive_time', 0.6)
        self.task = None
        self.t_end = 0.0
        self.pub_status = self.create_publisher(Task, 'task_status', RELIABLE)
        self.pub_phase = self.create_publisher(String, 'executor/phase', RELIABLE)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_nav', RELIABLE)
        self.create_service(AssignTask, 'assign_task', self.on_assign)
        self.create_timer(1.0 / self.param('rate', 20.0), self.tick)
        self.pub_phase.publish(String(data='idle'))

    def _status(self, status: int) -> None:
        msg = self.Task()
        msg.header.stamp = self.get_clock().now().to_msg()
        src = self.task
        msg.task_id = src.task_id if src else ''
        msg.robot_id = self.robot_id
        msg.priority = src.priority if src else 0
        msg.status = status
        self.pub_status.publish(msg)

    def _drive(self, v: float) -> None:
        cmd = Twist()
        cmd.linear.x = float(v)
        self.pub_cmd.publish(cmd)

    def on_assign(self, request, response):
        if self.task is not None:
            response.success = False
            response.robot_id = self.robot_id
            response.message = 'busy'
            return response
        self.task = request.task
        self.t_end = self.now() + self.drive_time
        self._drive(self.speed)                       # 첫 명령은 타이머를 기다리지 않는다
        self._status(self.Task.STATUS_IN_PROGRESS)
        self.pub_phase.publish(String(data='moving'))
        response.success = True
        response.robot_id = self.robot_id
        response.message = 'accepted'
        return response

    def tick(self) -> None:
        if self.task is None:
            return
        if self.now() < self.t_end:
            self._drive(self.speed)
            return
        self._drive(0.0)
        self._status(self.Task.STATUS_COMPLETED)
        self.pub_phase.publish(String(data='idle'))
        self.task = None


def main(args=None) -> None:
    run(TaskExecutor, args)


if __name__ == '__main__':
    main()
