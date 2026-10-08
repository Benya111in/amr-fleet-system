"""
분산 트리(task_executor_distributed.xml) ↔ 중앙 트리(task_executor.xml) 구조 대응 (보너스 4 b4-wire).

분산 트리는 중앙 트리의 사본이라 한쪽만 고치면 갈라진다 — 차이가 문서에 적은 셋뿐임을 고정한다.
1) include: yield.xml → yield_distributed.xml, zone_gates.xml 추가.
2) 메인 트리: 적재 도킹 바로 뒤 · Load 앞에 PickupLocked (task_lock_failed) 하나만 더 있다.
3) Yield 서브트리는 두 파일에서 같다. 분산 TrafficGate 는 구역 게이트(ZonePathClear)를 맨 앞에 두고, 중앙 hold 는
   IsFleetServerAlive 일 때만 따른다.
중앙 트리는 존 노드를 쓰지 않는다 (중앙 모드 실행기는 존 노드를 등록하지 않는다 — 쓰면 트리가 뜨지 않는다).
"""

from pathlib import Path
import xml.etree.ElementTree as ET

TREES = Path(__file__).resolve().parents[1] / 'behavior_trees'
ZONE_NODES = {'AcquireZoneChain', 'CommitTaskLock', 'IsFleetServerAlive', 'ReleaseZones',
              'ZoneGuard', 'ZonePathClear'}


def _root(name):
    return ET.parse(TREES / name).getroot()


def _tree(root, tree_id):
    found = [t for t in root.iter('BehaviorTree') if t.get('ID') == tree_id]
    assert len(found) == 1, tree_id
    return found[0]


def _same(a, b, path='') -> None:
    assert a.tag == b.tag, (path, a.tag, b.tag)
    assert a.attrib == b.attrib, (path, a.attrib, b.attrib)
    ca, cb = list(a), list(b)
    assert len(ca) == len(cb), (path, a.tag, [c.tag for c in ca], [c.tag for c in cb])
    for i, (x, y) in enumerate(zip(ca, cb)):
        _same(x, y, f'{path}/{a.tag}[{i}]')


def _includes(root):
    return [i.get('path') for i in root.findall('include')]


def test_includes_differ_only_in_the_gate_files():
    central = _includes(_root('task_executor.xml'))
    dist = _includes(_root('task_executor_distributed.xml'))
    assert 'subtrees/yield.xml' in central and 'subtrees/zone_gates.xml' not in central
    want = [p if p != 'subtrees/yield.xml' else 'subtrees/yield_distributed.xml' for p in central]
    assert dist == want + ['subtrees/zone_gates.xml']


def test_main_tree_differs_only_by_the_pickup_lock_step():
    central = _tree(_root('task_executor.xml'), 'TaskExecutor')
    dist = _tree(_root('task_executor_distributed.xml'), 'TaskExecutor')
    seq = [e for e in dist.iter('ResumableSequence') if e.get('name') == 'PickAndPlace']
    assert len(seq) == 1
    steps = list(seq[0])
    locks = [i for i, s in enumerate(steps) if s.get('reason') == 'task_lock_failed']
    assert len(locks) == 1
    k = locks[0]
    lock = steps[k]
    assert lock.tag == 'SetFailReason' and [c.get('ID') for c in lock] == ['PickupLocked']
    # 바로 앞은 적재 도킹, 바로 뒤는 적재
    assert steps[k - 1].get('reason') == 'dock_failed'
    assert [c.get('dock_id') for c in steps[k - 1]] == ['{pickup_dock_id}']
    assert steps[k + 1].get('reason') == 'load_failed'
    seq[0].remove(lock)
    _same(central, dist)                     # 나머지는 노드 · 속성까지 같다


def test_yield_is_shared_and_the_distributed_gate_checks_zones_first():
    _same(_tree(_root('subtrees/yield.xml'), 'Yield'),
          _tree(_root('subtrees/yield_distributed.xml'), 'Yield'))
    gate = _tree(_root('subtrees/yield_distributed.xml'), 'TrafficGate')
    top = list(gate)
    assert len(top) == 1 and top[0].tag == 'ReactiveSequence'
    first, second = list(top[0])
    assert first.get('name') == 'ZonePermission' and first[0].tag == 'ZonePathClear'
    assert second.get('name') == 'TrafficHoldCheck'
    tags = [c.tag for c in second]
    assert tags == ['Inverter', 'Inverter', 'SubTreePlus']
    assert second[0][0].tag == 'IsTrafficHold' and second[1][0].tag == 'IsFleetServerAlive'


def test_central_tree_files_do_not_use_zone_nodes():
    central = _root('task_executor.xml')
    files = ['task_executor.xml'] + _includes(central)
    for name in files:
        used = {e.tag for e in _root(name).iter()}
        assert not (used & ZONE_NODES), (name, used & ZONE_NODES)
    dist_used = {e.tag for e in _root('subtrees/yield_distributed.xml').iter()} | {
        e.tag for e in _root('subtrees/zone_gates.xml').iter()}
    assert {'ZonePathClear', 'IsFleetServerAlive', 'CommitTaskLock'} <= dist_used


def test_pickup_lock_uses_the_assigned_task_id():
    lock = _tree(_root('subtrees/zone_gates.xml'), 'PickupLocked')
    commit = list(lock)
    assert [c.tag for c in commit] == ['CommitTaskLock']
    assert commit[0].get('task_id') == '{task_id}'
    assert int(commit[0].get('timeout_ms')) > 0
