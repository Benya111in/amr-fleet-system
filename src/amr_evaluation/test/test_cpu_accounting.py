import math
import re

from amr_evaluation import cpu_accounting as acct
import pytest


def _stat(pid, comm, utime, stime, start):
    # /proc/<pid>/stat: pid (comm) state ppid ... utime(14) stime(15) ... starttime(22)
    fields = ['S'] + ['0'] * 10 + [str(utime), str(stime)] + ['0'] * 6 + [str(start)] + ['0'] * 5
    return f'{pid} ({comm}) ' + ' '.join(fields)


def _fake_proc(root, procs):
    for pid, comm, ut, st, start, cmd in procs:
        d = root / str(pid)
        d.mkdir(parents=True, exist_ok=True)
        (d / 'stat').write_text(_stat(pid, comm, ut, st, start))
        (d / 'cmdline').write_bytes(cmd.replace(' ', '\0').encode() + (b'\0' if cmd else b''))


def test_parse_groups():
    groups = acct.parse_groups(acct.DEFAULT_GROUPS)
    assert [g for g, _ in groups] == ['ros', 'gazebo', 'nav']
    ros, gz, nav = (p for _, p in groups)
    assert ros.search('/ros2_ws/install/amr_fleet/lib/amr_fleet/fleet_manager_node --ros-args')
    assert ros.search('ruby /usr/bin/ign gazebo -s -r warehouse.sdf')
    assert gz.search('ruby /usr/bin/ign gazebo -s -r warehouse.sdf')
    assert not gz.search('python3 /opt/ros/humble/bin/ros2 launch amr_simulation x.launch.py')
    assert nav.search('/opt/ros/humble/lib/rclcpp_components/component_container_isolated')
    assert not ros.search('/usr/bin/ansys --batch')
    with pytest.raises(ValueError):
        acct.parse_groups(['noequals'])
    with pytest.raises(ValueError):
        acct.parse_groups(['Bad-Name=x'])
    with pytest.raises(re.error):
        acct.parse_groups(['ok=('])


def test_parse_pid_stat_handles_spaces_in_comm():
    assert acct.parse_pid_stat(_stat(42, 'weird ) name', 30, 12, 999)) == (42, 999)
    assert acct.parse_pid_stat('garbage') is None
    assert acct.parse_pid_stat('1 (x) S 1 2') is None


def test_read_processes_and_cache(tmp_path):
    _fake_proc(tmp_path, [
        (10, 'gz', 100, 50, 1000, 'ruby /usr/bin/ign gazebo -s'),
        (11, 'kworker', 5, 5, 10, ''),                     # 커널 스레드: cmdline 없음 → 제외
        (12, 'nav', 30, 10, 2000,
         '/opt/ros/humble/lib/nav2_controller/controller_server --ros-args'),
    ])
    (tmp_path / 'self').mkdir()
    (tmp_path / '13').mkdir()                              # stat 없음 (끝난 프로세스)
    cache = {}
    procs = acct.read_processes(str(tmp_path), cache)
    assert sorted(p[0] for p in procs) == [10, 12]
    assert (10, 1000) in cache
    (tmp_path / '10' / 'cmdline').write_bytes(b'changed')
    procs = acct.read_processes(str(tmp_path), cache)       # 캐시된 cmdline 사용
    assert next(p for p in procs if p[0] == 10)[3].startswith('ruby')
    assert acct.read_processes(str(tmp_path / 'missing')) == []


def test_process_group_accounting():
    groups = acct.parse_groups(acct.DEFAULT_GROUPS)
    pga = acct.ProcessGroupAccounting(groups, clk_tck=100)
    assert pga.names == ['ros', 'gazebo', 'nav']
    gz = 'ruby /usr/bin/ign gazebo -s'
    nav = '/opt/ros/humble/lib/nav2_planner/planner_server --ros-args'
    other = '/usr/bin/ansys --batch'
    first = pga.update([(1, 10, 1000, gz), (2, 20, 500, nav), (3, 30, 9000, other)], 0.0, 4)
    assert math.isnan(first['ros'][0]) and first['ros'][1] == 2
    # 1 s 뒤: gz +200 tick (2 CPU·s), nav +50, 다른 사용자 +400, 새 ROS 프로세스 누적 30
    out = pga.update([(1, 10, 1200, gz), (2, 20, 550, nav), (3, 30, 9400, other),
                      (4, 40, 30, '/ros2_ws/install/x/lib/x/node --ros-args')], 1.0, 4)
    assert out['gazebo'] == (pytest.approx(50.0), 1)       # 2 CPU·s / (1 s × 4 CPU)
    assert out['nav'] == (pytest.approx(12.5), 1)
    assert out['ros'] == (pytest.approx(100.0 * 2.8 / 4), 3)
    # pid 가 재사용돼도 (starttime 다름) 새 프로세스로 본다
    out = pga.update([(1, 11, 5, gz)], 1.0, 4)
    assert out['gazebo'][0] == pytest.approx(100.0 * 0.05 / 4)
    assert math.isnan(pga.update([], 0.0, 4)['ros'][0])


def test_cgroup_usage(tmp_path):
    assert acct.parse_cgroup_v2_usage('usage_usec 2500000\nuser_usec 1\n') == 2.5
    assert acct.parse_cgroup_v2_usage('nr_periods 0\n') is None
    v2 = tmp_path / 'v2'
    v2.mkdir()
    (v2 / 'cpu.stat').write_text('usage_usec 1000000\n')
    assert acct.read_cgroup_usage(str(v2)) == 1.0
    v1 = tmp_path / 'v1' / 'cpu,cpuacct'
    v1.mkdir(parents=True)
    (v1 / 'cpuacct.usage').write_text('3000000000\n')
    assert acct.read_cgroup_usage(str(tmp_path / 'v1')) == 3.0
    assert acct.read_cgroup_usage(str(tmp_path / 'none')) is None
    assert acct.usage_percent(1.0, 3.0, 1.0, 4) == pytest.approx(50.0)
    assert math.isnan(acct.usage_percent(None, 3.0, 1.0, 4))
    assert math.isnan(acct.usage_percent(3.0, 1.0, 1.0, 4))
    assert math.isnan(acct.usage_percent(1.0, 3.0, 0.0, 4))
    assert acct.host_cpu_count() >= 1
