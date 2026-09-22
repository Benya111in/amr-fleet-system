"""rates · cdr · junit 순수 모듈."""

import math
import xml.etree.ElementTree as ET

from amr_itest import cdr, junit, rates
import numpy as np
import pytest


def test_rate_stats_regular_and_duplicates():
    stamps = [i * 0.1 for i in range(51)] + [4.0, float('nan')]   # 역행 1 + NaN 1
    st = rates.rate_stats(stamps)
    assert st.count == 51
    assert st.duplicates == 2
    assert st.mean_rate == pytest.approx(10.0)
    assert st.median_rate == pytest.approx(10.0)
    assert st.max_gap == pytest.approx(0.1)
    assert st.meets(10.0)
    assert not st.meets(12.0)
    assert set(st.as_dict()) >= {'mean_rate_hz', 'median_rate_hz', 'dropped_non_increasing'}


def test_rate_stats_drop_lowers_mean_not_median():
    stamps = [i * 0.01 for i in range(200) if i % 10 != 5]      # 10 % 드롭
    st = rates.rate_stats(stamps)
    assert st.median_rate == pytest.approx(100.0)
    assert st.mean_rate < 100.0
    assert st.max_gap == pytest.approx(0.02)


def test_rate_stats_degenerate():
    assert rates.rate_stats([]).count == 0
    assert not rates.rate_stats([1.0]).meets(10.0)
    assert not rates.rate_stats([0.0, 1.0]).meets(0.0)
    assert rates.in_window([0.0, 1.0, 2.0, 3.0], 1.0, 2.0) == [1.0, 2.0]


def test_sample_std_and_beam_noise():
    assert math.isnan(rates.sample_std([1.0]))
    assert rates.sample_std([1.0, 3.0]) == pytest.approx(math.sqrt(2.0))
    rng = np.random.default_rng(1)
    base = np.linspace(1.0, 5.0, 100)
    scans = [base + rng.normal(0, 0.03, 100) for _ in range(40)]
    scans = [np.where(np.arange(100) < 10, np.inf, s) for s in scans]   # 미검출 빔 제외
    assert rates.per_beam_noise(scans, 25.0) == pytest.approx(0.03, rel=0.2)
    assert math.isnan(rates.per_beam_noise(scans[:1], 25.0))
    assert math.isnan(rates.per_beam_noise([[np.inf] * 3, [np.inf] * 3], 25.0))


@pytest.mark.parametrize('little', [True, False])
def test_cdr_roundtrip(little):
    data = cdr.encode_header(12, 500_000_000, 'amr_01/lidar_link', little)
    stamp, frame = cdr.header(data)
    assert stamp == pytest.approx(12.5)
    assert frame == 'amr_01/lidar_link'
    assert cdr.header_stamp(data) == pytest.approx(12.5)


def test_cdr_errors():
    with pytest.raises(ValueError):
        cdr.header(b'\x00')
    with pytest.raises(ValueError):
        cdr.header(b'\x00\x05\x00\x00' + b'\x00' * 20)       # 알 수 없는 encapsulation
    with pytest.raises(ValueError):
        cdr.header_stamp(b'\x00\x01\x00\x00\x01')             # 스탬프 잘림
    with pytest.raises(ValueError):
        cdr.header(cdr.encode_header(1, 0, 'x')[:14])         # 길이 필드 잘림
    trunc = cdr.encode_header(1, 0, 'abcdef')[:-3]
    with pytest.raises(ValueError):
        cdr.header(trunc)
    empty = b'\x00\x01\x00\x00' + (1).to_bytes(4, 'little') + (0).to_bytes(4, 'little') \
        + (0).to_bytes(4, 'little')
    assert cdr.header(empty) == (1.0, '')


def _write(path, cases, name='suite'):
    root = ET.Element('testsuites')
    s = ET.SubElement(root, 'testsuite', name=name, time='1.5')
    for cname, tag in cases:
        c = ET.SubElement(s, 'testcase', name=cname, classname=name, time='0.5')
        if tag:
            ET.SubElement(c, tag, message=f'{cname} {tag}')
    ET.ElementTree(root).write(str(path))
    return path


def test_junit_status_and_aggregate(tmp_path):
    ok = _write(tmp_path / 'ok.xml', [('a', None), ('b', 'skipped')])
    fail = _write(tmp_path / 'fail.xml', [('a', 'failure'), ('b', None)])
    err = _write(tmp_path / 'err.xml', [('a', 'error')])
    skip = _write(tmp_path / 'skip.xml', [('a', 'skipped'), ('b', 'skipped')])
    # 일부 skip 은 통과가 아니다: partial (report 가 needs 설치 여부로 실패로 셀지 정한다)
    assert junit.status_of(ok) == junit.PARTIAL
    full = _write(tmp_path / 'full.xml', [('a', None), ('b', None)])
    assert junit.status_of(full) == junit.PASSED
    assert junit.case_counts(ok) == {'total': 2, 'executed': 1, 'passed': 1, 'failed': 0,
                                     'error': 0, 'skipped': 1}
    assert junit.case_counts(None)['total'] == 0
    assert junit.case_counts(tmp_path / 'none.xml')['executed'] == 0
    assert junit.status_of(fail) == junit.FAILED
    assert junit.status_of(err) == junit.ERROR
    assert junit.status_of(skip) == junit.SKIPPED
    assert junit.status_of(tmp_path / 'none.xml') == junit.MISSING
    assert junit.status_of(None) == junit.MISSING
    (tmp_path / 'bad.xml').write_text('<testsuites')
    assert junit.status_of(tmp_path / 'bad.xml') == junit.ERROR
    assert junit.case_counts(tmp_path / 'bad.xml')['total'] == 0
    (tmp_path / 'empty.xml').write_text('<testsuites/>')
    assert junit.status_of(tmp_path / 'empty.xml') == junit.ERROR
    assert junit.skip_message(skip) == 'a skipped'
    assert junit.skip_message(tmp_path / 'bad.xml') == ''
    totals = junit.aggregate([ok, fail, err, skip, tmp_path / 'none.xml', tmp_path / 'bad.xml'],
                             tmp_path / 'all.xml')
    assert totals == {'tests': 7, 'failures': 1, 'errors': 1, 'skipped': 3, 'time': 6.0}
    root = ET.parse(str(tmp_path / 'all.xml')).getroot()
    assert root.get('tests') == '7' and len(root.findall('testsuite')) == 4


def test_junit_single_suite_root_and_empty_suite(tmp_path):
    p = tmp_path / 's.xml'
    p.write_text('<testsuite name="x"><testcase name="t"/></testsuite>')
    assert [s.name for s in junit.parse(p)] == ['x']
    assert junit.SuiteResult('empty').status == junit.ERROR


@pytest.mark.parametrize('status', [junit.FAILED, junit.ERROR, junit.SKIPPED, junit.PASSED])
def test_junit_synthetic(tmp_path, status):
    path = junit.synthetic(tmp_path / 'd' / 'junit.xml', 'suite', 'case', status, 'why', 3.0)
    assert junit.status_of(path) == status
