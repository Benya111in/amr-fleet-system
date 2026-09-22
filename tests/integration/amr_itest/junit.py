"""
JUnit XML 읽기·합치기 (rclpy 비의존 순수 모듈).

launch_test --junit-xml 은 시나리오 파일마다 <testsuites><testsuite>...</testsuite></testsuites>
를 쓴다. 이것들을 하나의 <testsuites> 로 합쳐 CI(Jenkins/GitLab/GitHub 액션의 JUnit 리포터)가
한 번에 읽게 하고, 시나리오별 상태(passed/failed/error/skipped)를 뽑는다.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, List, Optional
import xml.etree.ElementTree as ET

PASSED = 'passed'
FAILED = 'failed'
ERROR = 'error'
SKIPPED = 'skipped'
MISSING = 'missing'     # JUnit 파일이 없음 (타임아웃·크래시로 launch_test 가 쓰지 못함)


@dataclass
class CaseResult:
    """testcase 하나."""

    name: str
    classname: str = ''
    time: float = 0.0
    status: str = PASSED
    message: str = ''


@dataclass
class SuiteResult:
    """testsuite 하나 (= 시나리오 파일 하나의 테스트 런)."""

    name: str
    cases: List[CaseResult] = field(default_factory=list)
    time: float = 0.0

    def count(self, status: str) -> int:
        return sum(1 for c in self.cases if c.status == status)

    @property
    def status(self) -> str:
        """스위트 상태: 실패/에러가 하나라도 있으면 그것, 전부 skip 이면 skipped."""
        if not self.cases:
            return ERROR
        if self.count(ERROR):
            return ERROR
        if self.count(FAILED):
            return FAILED
        if self.count(SKIPPED) == len(self.cases):
            return SKIPPED
        return PASSED


def _case(elem: ET.Element) -> CaseResult:
    case = CaseResult(name=elem.get('name', ''), classname=elem.get('classname', ''),
                      time=float(elem.get('time', 0.0) or 0.0))
    for tag, status in (('error', ERROR), ('failure', FAILED), ('skipped', SKIPPED)):
        child = elem.find(tag)
        if child is not None:
            case.status = status
            case.message = (child.get('message') or child.text or '').strip()
            break
    return case


def parse(path: Path) -> List[SuiteResult]:
    """파일 하나의 JUnit XML → 스위트 목록 (루트가 testsuites 든 testsuite 든)."""
    root = ET.parse(str(path)).getroot()
    suites = [root] if root.tag == 'testsuite' else root.findall('testsuite')
    out = []
    for s in suites:
        suite = SuiteResult(name=s.get('name', path.stem),
                            time=float(s.get('time', 0.0) or 0.0))
        suite.cases = [_case(c) for c in s.findall('testcase')]
        out.append(suite)
    return out


def status_of(path: Optional[Path]) -> str:
    """시나리오 JUnit 파일 하나의 종합 상태 (없거나 깨졌으면 missing/error)."""
    if path is None or not Path(path).is_file():
        return MISSING
    try:
        suites = parse(Path(path))
    except ET.ParseError:
        return ERROR
    if not suites:
        return ERROR
    statuses = [s.status for s in suites]
    for st in (ERROR, FAILED):
        if st in statuses:
            return st
    if all(st == SKIPPED for st in statuses):
        return SKIPPED
    return PASSED


def skip_message(path: Path) -> str:
    """스킵된 시나리오의 사유 (첫 skipped 메시지)."""
    try:
        for suite in parse(Path(path)):
            for c in suite.cases:
                if c.status == SKIPPED and c.message:
                    return c.message
    except (ET.ParseError, OSError):
        pass
    return ''


def aggregate(paths: Iterable[Path], out_path: Path, name: str = 'amr_integration') -> dict:
    """
    여러 JUnit 파일의 testsuite 를 하나의 <testsuites> 로 합쳐 out_path 에 쓴다.

    반환: {'tests', 'failures', 'errors', 'skipped', 'time'} 합계.
    """
    root = ET.Element('testsuites', name=name)
    totals = {'tests': 0, 'failures': 0, 'errors': 0, 'skipped': 0, 'time': 0.0}
    for path in paths:
        path = Path(path)
        if not path.is_file():
            continue
        try:
            src = ET.parse(str(path)).getroot()
        except ET.ParseError:
            continue
        suites = [src] if src.tag == 'testsuite' else src.findall('testsuite')
        for s in suites:
            root.append(s)
            cases = s.findall('testcase')
            totals['tests'] += len(cases)
            totals['failures'] += sum(1 for c in cases if c.find('failure') is not None)
            totals['errors'] += sum(1 for c in cases if c.find('error') is not None)
            totals['skipped'] += sum(1 for c in cases if c.find('skipped') is not None)
            totals['time'] += float(s.get('time', 0.0) or 0.0)
    for key in ('tests', 'failures', 'errors', 'skipped'):
        root.set(key, str(totals[key]))
    root.set('time', f"{totals['time']:.3f}")
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(str(out_path), encoding='utf-8', xml_declaration=True)
    return totals


def synthetic(out_path: Path, suite: str, case: str, status: str, message: str = '',
              time: float = 0.0) -> Path:
    """
    launch_test 가 JUnit 을 못 남긴 경우(타임아웃·크래시)를 대신 기록한다.

    status 는 failed/error/skipped/passed. 집계에서 시나리오가 사라지지 않게 하려는 것이다.
    """
    root = ET.Element('testsuites')
    s = ET.SubElement(root, 'testsuite', name=suite, tests='1', time=f'{time:.3f}',
                      failures='1' if status == FAILED else '0',
                      errors='1' if status == ERROR else '0',
                      skipped='1' if status == SKIPPED else '0')
    c = ET.SubElement(s, 'testcase', name=case, classname=suite, time=f'{time:.3f}')
    if status in (FAILED, ERROR, SKIPPED):
        tag = {FAILED: 'failure', ERROR: 'error', SKIPPED: 'skipped'}[status]
        ET.SubElement(c, tag, message=message).text = message
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(str(out_path), encoding='utf-8', xml_declaration=True)
    return out_path
