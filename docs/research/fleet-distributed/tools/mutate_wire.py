#!/usr/bin/env python3
"""
b4-wire 연결 변이 검사 — 새 연결 · 가드를 하나씩 되돌린 사본에서 지정 시험이 실패하는지 본다 (COORDINATION §4).

컨테이너 안에서 (저장소 스냅숏을 /src 로 읽기 전용, 산출 /out):
  python3 /src/docs/research/fleet-distributed/tools/mutate_wire.py --out /out [--only M-HB,M-REG]
1) 기준 빌드: /src 사본을 /tmp/mw/base 에 두고 amr_msgs amr_fleet amr_perception amr_behavior 를 빌드 (Release).
2) 변이마다 /tmp/mw/<id> 사본 + 문자열 치환(정확히 1 회 일치해야 함 — 아니면 BAD_MUTANT). C++/트리 변이는 그 사본의
   amr_behavior 만 기준 설치 위에 다시 빌드한다. 지정 시험 명령이 0 이 아니면 KILLED (잡힘), 0 이면 SURVIVED.
변이 목록 · 기대(잡을 시험)는 아래 MUTATIONS — 결과 전에 커밋한다 (사전 등록).
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

FM = 'src/amr_fleet/amr_fleet/fleet_manager_node.py'
HB = 'src/amr_fleet/amr_fleet/heartbeat.py'
NODE = 'src/amr_fleet/amr_fleet/agent/node.py'
CORE = 'src/amr_fleet/amr_fleet/agent/core.py'
EXEC = 'src/amr_behavior/src/task_executor_node.cpp'
LAUNCH = 'src/amr_behavior/launch/behavior.launch.py'
LINK = 'src/amr_behavior/include/amr_behavior/bt_nodes/zone_agent_link.hpp'
DIST = 'src/amr_behavior/behavior_trees/task_executor_distributed.xml'
YDIST = 'src/amr_behavior/behavior_trees/subtrees/yield_distributed.xml'

# PYTHONPATH 는 덧붙인다 — 덮어쓰면 rclpy · amr_msgs 를 못 찾아 노드 시험이 건너뛰어진다 (W-mut 1 회차의 M-HB 가
# 그렇게 공허하게 SURVIVED — 결과 뒤 변경, 노트에 적음). 시험이 하나도 통과 · 실패하지 않으면 INVALID 로 본다.
PY_FLEET = ('cd {d}/src/amr_fleet && PYTHONPATH={d}/src/amr_fleet:$PYTHONPATH '
            'python3 -m pytest -q -x -p no:cacheprovider {t}')
PY_BEH = 'cd {d}/src/amr_behavior && python3 -m pytest -q -x -p no:cacheprovider {t}'
GTEST = 'cd {d}/ws/build/amr_behavior && ctest --output-on-failure -R "^{t}$"'

W1 = ('cd {d}/src/amr_behavior/test_fleet_world && PYTHONPATH={d}/src/amr_fleet:$PYTHONPATH '
      'TASK_EXECUTOR_EXE=/tmp/mw/base/ws/build/amr_behavior/task_executor_node '
      'AMR_WIRE_RAW=/tmp/mw/{t}_raw python3 -m pytest -q -x -p no:cacheprovider test_wire_launch.py'
      ' ; rc=$? ; cp -r /tmp/mw/{t}_raw $OUT_DIR/ 2>/dev/null ; exit $rc')

# (id, 설명, 파일, [(원문, 치환)], 빌드 필요, 시험 명령 틀, 시험)
MUTATIONS = [
    ('M-HB', '관리자가 하트비트를 내지 않는다 (발행 삭제)', FM,
     [('        self._pub_hb.publish(msg)', '        pass  # 변이 M-HB')], False, PY_FLEET,
     'test/test_manager_heartbeat.py'),
    ('M-OBS', '관찰자 복귀가 로봇이 본 최고 epoch + 1 을 쓰지 않는다', HB,
     [('            self.epoch = max(self.base_epoch, self.max_seen + 1)',
       '            self.epoch = self.base_epoch')], False, PY_FLEET,
     'test/test_heartbeat.py test/test_manager_heartbeat.py'),
    ('M-REG', '분산 모드 실행기가 존 노드를 등록하지 않는다', EXEC,
     [('    fleet_link_ = registerFleetAgentNodes(factory_, ros);', '    fleet_link_ = nullptr;')],
     True, GTEST, 'test_task_executor_node'),
    ('M-REMAP', '분산 모드 런치가 실행기 assign_task 를 리매핑하지 않는다', LAUNCH,
     [("            remappings=[('assign_task', 'executor/assign_task')] if distributed else []))",
       '            remappings=[]))')], False, PY_BEH, 'test/test_config_launch.py'),
    ('M-PL', '분산 트리에서 PickupLocked 단계 삭제', DIST,
     [('''                  <SetFailReason reason="task_lock_failed" fail_reason="{fail_reason}">
                    <SubTreePlus ID="PickupLocked" __autoremap="true"/>
                  </SetFailReason>
''', '')], True, GTEST, 'test_task_tree_distributed'),
    ('M-SRV', '분산 TrafficGate 가 서버 생존과 무관하게 hold 를 따른다', YDIST,
     [('''        <Inverter>
          <IsFleetServerAlive/>
        </Inverter>
''', '')], True, GTEST, 'test_task_tree_distributed'),
    ('M-FRESH', '미래 stamp 허용치 삭제 (age < 0 도 신선)', LINK,
     [('    if (age > rclcpp::Duration(max_age_) || age < rclcpp::Duration(-future_tolerance_)) {',
       '    if (age > rclcpp::Duration(max_age_)) {')], True, GTEST, 'test_zone_bt_nodes'),
    ('M-SIMCLK', '존 노드 시간 초과를 벽시계로', LINK,
     [('  rclcpp::Time now() const {return clock_->now();}',
       '  rclcpp::Time now() const {return rclcpp::Clock(RCL_STEADY_TIME).now();}')],
     True, GTEST, 'test_zone_bt_nodes'),
    ('M-GC', '협상 표 낡은 LOCKED 정리를 하지 않는다', CORE,
     [('        seen = set()\n        for task, bid in list(self.cbaa.table.items()):',
       '        return\n        seen = set()\n        for task, bid in list(self.cbaa.table.items()):')],
     False, PY_FLEET, 'test/test_agent_dispatch.py'),
    ('M-PLAN', '에이전트 노드가 plan 을 경로 게이트에 넣지 않는다 (ROS 결선 — launch W1 이 잡아야)',
     NODE, [('        self.core.set_plan(pts if len(pts) >= 2 else None)',
             '        self.core.set_plan(None)')], False, W1, 'M-PLAN'),
    # 결과 뒤 추가 (W-diag B 에서 찾은 거짓 양성을 고친 가드, 이 변이는 그 고침과 같은 커밋에서 등록)
    ('M-STAMP', '낡은 LOCKED 정리가 LOCKED 를 본 뒤 만든 비컨인지 보지 않는다', CORE,
     [('            made_after = b.stamp > since + self.cfg.stale_lock_margin_s',
       '            made_after = True')], False, PY_FLEET, 'test/test_agent_dispatch.py'),
    ('M-GATE', '경로 게이트가 결정점 안에서도 막지 않는다 (blocking 비움)',
     'src/amr_fleet/amr_fleet/agent/path_gate.py',
     [('            blocking = tuple(r for r in request if r not in held)',
       '            blocking = ()')], False, PY_FLEET, 'test/test_agent_path_gate.py'),
]


def sh(cmd, log, timeout=3600):
    with open(log, 'a', encoding='utf-8') as f:
        f.write(f'$ {cmd}\n')
        f.flush()
        p = subprocess.run(['bash', '-c', cmd], stdout=f, stderr=subprocess.STDOUT,
                           timeout=timeout)
    return p.returncode


def copy_tree(src, dst):
    if os.path.exists(dst):
        shutil.rmtree(dst)
    shutil.copytree(src, dst, ignore=shutil.ignore_patterns('build', 'install', 'log', '.git'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--only', default='')
    a = ap.parse_args()
    only = set(filter(None, a.only.split(',')))
    os.makedirs(a.out, exist_ok=True)
    base = '/tmp/mw/base'
    copy_tree('/src', base)
    blog = os.path.join(a.out, 'base_build.log')
    ros = 'source /opt/ros/humble/setup.bash'
    rc = sh(f'{ros} && cd {base} && MAKEFLAGS=-j4 colcon --log-base {base}/ws/log build '
            f'--build-base {base}/ws/build --install-base {base}/ws/install --parallel-workers 4 '
            '--packages-up-to amr_msgs amr_fleet amr_perception amr_behavior '
            '--cmake-args -DCMAKE_BUILD_TYPE=Release', blog)
    results = {'base_build_rc': rc, 'mutations': []}
    if rc != 0:
        json.dump(results, open(os.path.join(a.out, 'results.json'), 'w'), indent=1)
        return 1
    for mid, desc, path, subs, build, tmpl, tests in MUTATIONS:
        if only and mid not in only:
            continue
        d = f'/tmp/mw/{mid}'
        copy_tree('/src', d)
        log = os.path.join(a.out, f'{mid}.log')
        src = open(os.path.join(d, path), encoding='utf-8').read()
        bad = [old for old, _ in subs if src.count(old) != 1]
        entry = {'id': mid, 'desc': desc, 'file': path, 'tests': tests}
        if bad:
            entry['verdict'] = 'BAD_MUTANT'
            entry['why'] = f'일치 수 ≠ 1: {bad[0][:60]!r}'
            results['mutations'].append(entry)
            continue
        for old, new in subs:
            src = src.replace(old, new)
        open(os.path.join(d, path), 'w', encoding='utf-8').write(src)
        t0 = time.time()
        env = f'{ros} && source {base}/ws/install/setup.bash'
        if build:
            brc = sh(f'{env} && cd {d} && MAKEFLAGS=-j4 colcon --log-base {d}/ws/log build '
                     f'--build-base {d}/ws/build --install-base {d}/ws/install '
                     '--packages-select amr_behavior --cmake-args -DCMAKE_BUILD_TYPE=Release',
                     log)
            if brc != 0:
                entry.update(verdict='BAD_MUTANT', why='빌드 실패')
                results['mutations'].append(entry)
                continue
            env += f' && source {d}/ws/install/setup.bash'
        dom = os.environ.get('MW_DOMAIN', '59')
        trc = sh(f'{env} && export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID={dom} OUT_DIR={a.out} '
                 f'&& {tmpl.format(d=d, t=tests)}', log, timeout=5400)
        tail = open(log, encoding='utf-8', errors='replace').read()[-3000:]
        ran = any(w in tail for w in ('passed', 'failed', 'Passed', 'Failed', 'error'))
        entry.update(test_rc=trc, wall_s=round(time.time() - t0, 1),
                     verdict=('KILLED' if trc != 0 else 'SURVIVED') if ran else 'INVALID')
        if not ran:
            entry['why'] = '시험이 돌지 않았다 (모두 건너뜀 · 수집 0)'
        results['mutations'].append(entry)
        shutil.rmtree(d, ignore_errors=True)
        json.dump(results, open(os.path.join(a.out, 'results.json'), 'w'), indent=1,
                  ensure_ascii=False)
    json.dump(results, open(os.path.join(a.out, 'results.json'), 'w'), indent=1,
              ensure_ascii=False)
    for m in results['mutations']:
        print(f"{m['id']:9s} {m['verdict']:10s} {m['desc']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
