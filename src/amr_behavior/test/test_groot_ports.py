"""
Groot ZMQ 포트 시험 (리뷰 회귀): 한 호스트의 실행기 여러 대가 로봇마다 다른 포트로 동시에 bind 한다.

amr_bringup launch_utils.stack_arguments 는 로봇 i(0부터)에 groot_publisher_port = 1666 + 2i,
groot_server_port = 1667 + 2i 를 주고 behavior.launch.py 가 실행기 파라미터로 넘긴다. BT.CPP v3 의
PublisherZMQ 는 프로세스당 1 개라 프로세스를 나눠 실제 실행 파일로 확인한다 (시험 포트는 17766 부터).
같은 포트를 또 쓰면 그 실행기만 경고 후 Groot 없이 계속한다.
"""

import os
import socket
import subprocess
import threading
import time
import uuid

import pytest

EXE = os.environ.get('TASK_EXECUTOR_EXE', '')
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XML = os.path.join(PKG, 'behavior_trees', 'task_executor.xml')
BASE = 17766


class _Executor:
    def __init__(self, ns, pub, srv):
        env = dict(os.environ, RCUTILS_LOGGING_BUFFERED_STREAM='0', RCUTILS_LOGGING_USE_STDOUT='1')
        self.proc = subprocess.Popen(
            [EXE, '--ros-args', '-r', f'__ns:=/{ns}', '-p', f'bt_xml:={XML}',
             '-p', f'groot.publisher_port:={pub}', '-p', f'groot.server_port:={srv}',
             '-p', 'charger_claims_topic:=""'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, text=True)
        self.lines = []
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        for line in self.proc.stdout:
            self.lines.append(line)

    def wait_for(self, text, timeout=30.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if any(text in line for line in self.lines):
                return True
            time.sleep(0.05)
        return False

    def stop(self):
        self.proc.terminate()
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:  # pragma: no cover
            self.proc.kill()


def _listening(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(1.0)
        return s.connect_ex(('127.0.0.1', port)) == 0


@pytest.mark.skipif(not EXE or not os.path.exists(EXE), reason='TASK_EXECUTOR_EXE 가 없다')
def test_each_robot_binds_its_own_groot_ports():
    tag = uuid.uuid4().hex[:6]
    robots = []
    try:
        for i in range(3):                      # amr_01..03 → bringup 규칙 BASE + 2i / BASE + 2i + 1
            robots.append(_Executor(f'g{tag}/amr_0{i + 1}', BASE + 2 * i, BASE + 2 * i + 1))
        for i, r in enumerate(robots):
            assert r.wait_for(f'Groot ZMQ: publisher :{BASE + 2 * i}'), ''.join(r.lines[-20:])
            assert _listening(BASE + 2 * i) and _listening(BASE + 2 * i + 1)
        clash = _Executor(f'g{tag}/amr_09', BASE, BASE + 1)    # 로봇 0 과 같은 포트
        robots.append(clash)
        assert clash.wait_for('Groot ZMQ 퍼블리셔를 열 수 없다'), ''.join(clash.lines[-20:])
        assert clash.wait_for('phase → IDLE')               # 실행기 자체는 계속 돈다
        assert robots[0].proc.poll() is None
    finally:
        for r in robots:
            r.stop()
