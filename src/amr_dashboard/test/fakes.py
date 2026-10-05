"""
테스트용 메시지 생성기: duck-typed 대역(항상)과 실제 ROS 메시지(임포트 가능할 때).

state_store 는 속성 접근만 쓰므로 두 종류를 같은 테스트로 검증한다. 실제 메시지는 colcon test
(amr_msgs 빌드 후) 환경에서만 있으므로 HAVE_ROS 로 분기한다.
"""

import math
from types import SimpleNamespace as NS

try:
    from amr_msgs.msg import FleetStatus, RobotState, Task
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import OccupancyGrid
    HAVE_ROS = True
except ImportError:  # ROS 환경 밖 (순수 pytest)
    HAVE_ROS = False


def yaw_to_quat(yaw):
    """요(yaw) [rad] → 쿼터니언 (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


# ----- duck-typed 대역 -----

def fake_stamp(sec=100, nanosec=500_000_000):
    return NS(sec=sec, nanosec=nanosec)


def fake_pose(x=1.0, y=2.0, yaw=0.5, frame_id='map'):
    qx, qy, qz, qw = yaw_to_quat(yaw)
    return NS(header=NS(stamp=fake_stamp(), frame_id=frame_id),
              pose=NS(position=NS(x=x, y=y, z=0.0), orientation=NS(x=qx, y=qy, z=qz, w=qw)))


def fake_robot_state(robot_id='amr_01', x=1.0, y=2.0, yaw=0.5, battery=87.5,
                     task_id='T-1', status=1):
    return NS(header=NS(stamp=fake_stamp(), frame_id='map'), robot_id=robot_id,
              pose=fake_pose(x, y, yaw), battery_level=battery, current_task_id=task_id,
              status=status)


def fake_fleet_status(robots=None, pending=3, in_progress=2, completed=10, failed=1,
                      throughput=12.5, avg_duration=48.0, utilization=0.8, deadlocks=0):
    if robots is None:
        robots = [fake_robot_state('amr_01'),
                  fake_robot_state('amr_02', 3.0, 4.0, -1.0, 40.0, '', 0)]
    return NS(header=NS(stamp=fake_stamp(), frame_id=''), robots=robots,
              tasks_pending=pending, tasks_in_progress=in_progress, tasks_completed=completed,
              tasks_failed=failed, throughput=throughput, avg_task_duration=avg_duration,
              robot_utilization=utilization, deadlock_count=deadlocks)


def fake_diag_array(level=b'\x02', name='task_failed', message='도킹 3회 실패',
                    hardware_id='amr_03', values=None):
    kvs = [NS(key=k, value=v) for k, v in (values or {'task_id': 'T-9'}).items()]
    status = NS(level=level, name=name, message=message, hardware_id=hardware_id, values=kvs)
    return NS(header=NS(stamp=fake_stamp(), frame_id=''), status=[status])


def fake_task(task_id='T-1', robot_id='amr_01', priority=200, status=1, item_type='medium',
              item_mass=10.0, deadline_sec=1000):
    return NS(header=NS(stamp=fake_stamp(), frame_id=''), task_id=task_id, robot_id=robot_id,
              priority=priority, deadline=fake_stamp(deadline_sec, 0),
              pickup_pose=fake_pose(5.0, 5.0, 0.0), dropoff_pose=fake_pose(50.0, 30.0, 1.5),
              item_type=item_type, item_mass=item_mass, status=status)


def fake_grid(width=4, height=3, resolution=0.5, origin=(1.0, 2.0), data=None):
    if data is None:
        data = [0] * (width * height)
        data[0] = 100
        data[-1] = -1
    return NS(header=NS(stamp=fake_stamp(7, 0), frame_id='map'),
              info=NS(width=width, height=height, resolution=resolution,
                      origin=NS(position=NS(x=origin[0], y=origin[1], z=0.0),
                                orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))),
              data=data)


# ----- 실제 ROS 메시지 -----

def _fill_pose(pose_stamped, x, y, yaw, frame_id='map'):
    pose_stamped.header.frame_id = frame_id
    pose_stamped.header.stamp.sec = 100
    pose_stamped.header.stamp.nanosec = 500_000_000
    pose_stamped.pose.position.x = x
    pose_stamped.pose.position.y = y
    qx, qy, qz, qw = yaw_to_quat(yaw)
    pose_stamped.pose.orientation.x = qx
    pose_stamped.pose.orientation.y = qy
    pose_stamped.pose.orientation.z = qz
    pose_stamped.pose.orientation.w = qw
    return pose_stamped


def real_robot_state(robot_id='amr_01', x=1.0, y=2.0, yaw=0.5, battery=87.5,
                     task_id='T-1', status=1):
    msg = RobotState()
    msg.header.stamp.sec = 100
    msg.header.stamp.nanosec = 500_000_000
    msg.robot_id = robot_id
    _fill_pose(msg.pose, x, y, yaw)
    msg.battery_level = battery
    msg.current_task_id = task_id
    msg.status = status
    return msg


def real_fleet_status(robots=None, pending=3, in_progress=2, completed=10, failed=1,
                      throughput=12.5, avg_duration=48.0, utilization=0.8, deadlocks=0):
    msg = FleetStatus()
    msg.header.stamp.sec = 100
    msg.header.stamp.nanosec = 500_000_000
    if robots is None:
        robots = [real_robot_state('amr_01'),
                  real_robot_state('amr_02', 3.0, 4.0, -1.0, 40.0, '', 0)]
    msg.robots = robots
    msg.tasks_pending = pending
    msg.tasks_in_progress = in_progress
    msg.tasks_completed = completed
    msg.tasks_failed = failed
    msg.throughput = throughput
    msg.avg_task_duration = avg_duration
    msg.robot_utilization = utilization
    msg.deadlock_count = deadlocks
    return msg


def real_diag_array(level=b'\x02', name='task_failed', message='도킹 3회 실패',
                    hardware_id='amr_03', values=None):
    msg = DiagnosticArray()
    msg.header.stamp.sec = 100
    msg.header.stamp.nanosec = 500_000_000
    st = DiagnosticStatus()
    st.level = level
    st.name = name
    st.message = message
    st.hardware_id = hardware_id
    st.values = [KeyValue(key=k, value=v) for k, v in (values or {'task_id': 'T-9'}).items()]
    msg.status = [st]
    return msg


def real_task(task_id='T-1', robot_id='amr_01', priority=200, status=1, item_type='medium',
              item_mass=10.0, deadline_sec=1000):
    msg = Task()
    msg.header.stamp.sec = 100
    msg.header.stamp.nanosec = 500_000_000
    msg.task_id = task_id
    msg.robot_id = robot_id
    msg.priority = priority
    msg.deadline.sec = deadline_sec
    _fill_pose(msg.pickup_pose, 5.0, 5.0, 0.0)
    _fill_pose(msg.dropoff_pose, 50.0, 30.0, 1.5)
    msg.item_type = item_type
    msg.item_mass = item_mass
    msg.status = status
    return msg


def real_grid(width=4, height=3, resolution=0.5, origin=(1.0, 2.0), data=None):
    msg = OccupancyGrid()
    msg.header.stamp.sec = 7
    msg.header.frame_id = 'map'
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    msg.info.origin.position.x = origin[0]
    msg.info.origin.position.y = origin[1]
    if data is None:
        data = [0] * (width * height)
        data[0] = 100
        data[-1] = -1
    msg.data = data
    return msg


FAKE = {
    'robot_state': fake_robot_state, 'fleet_status': fake_fleet_status,
    'diag_array': fake_diag_array, 'task': fake_task, 'grid': fake_grid,
}
REAL = {
    'robot_state': real_robot_state, 'fleet_status': real_fleet_status,
    'diag_array': real_diag_array, 'task': real_task, 'grid': real_grid,
}
