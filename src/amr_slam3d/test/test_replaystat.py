"""재생 산출물 요약 시험 (amr_slam3d.replaystat) — 처리 수 근거·누락 계수·manifest 항목."""

import json

from amr_slam3d import replaystat
import pytest


def _w(d, name, obj):
    p = d / name
    p.write_text(obj if isinstance(obj, str) else json.dumps(obj), encoding='utf-8')


def test_family_and_unknown():
    assert replaystat.family('rtabmap3d_noimu') == 'rtabmap'
    assert replaystat.family('liosam_planar') == 'liosam'
    assert replaystat.family('slam_toolbox') == 'slam_toolbox'
    with pytest.raises(ValueError):
        replaystat.family('cartographer')


def test_parse_env_and_db_info():
    env = ('date: x\nplay_start_wall: 2026-10-06T22:00:00+09:00 loadavg: 3.50 1 1 1/1 1\n'
           'play_end_wall: 2026-10-06T22:06:00+09:00 loadavg: 7.25 1 1 1/1 1\n'
           'launch_exit: clean after 4s\n')
    e = replaystat.parse_env(env)
    assert e == {'launch_exit': 'clean after 4s', 'load_play_start': 3.5, 'load_play_end': 7.25}
    assert replaystat.parse_env('')['launch_exit'] is None
    assert replaystat.optimized_poses('Optimized graph:    0 poses (x=310->0)') == 0
    assert replaystat.optimized_poses('Optimized graph:  139 poses') == 139
    assert replaystat.optimized_poses('nothing') is None


def _rtab(d, probe_n, log_lines, bag_n=1788, judged=None, match=True):
    _w(d, 'probe.json', {'input': {'count': float(probe_n)}, 'traj_online': {'n': 1786},
                         'tf': {'valid': True}})
    _w(d, 'log_scan.json', {'fatal': {'a': 0, 'b': 0},
                            'note': {"didn't receive IMU newer": 1,
                                     'Odom: ratio=0.000000': 1},
                            'common': {}, 'processed_log_lines': log_lines})
    if match:
        miss = [9.5, 30.0][:judged] if judged is not None else []
        _w(d, 'match_amr_01_icp_odom.json',
           {'input': float(bag_n), 'missing': float(judged), 'judged_input': float(bag_n),
            'judged_missing': float(judged), 'judged_missing_stamps': miss})


def test_summarize_rtabmap_counts_log_lines(tmp_path):
    """
    RTAB-Map 누락 분모 = bag 장 수 (ratio=0 줄은 처리로 세되 null_odom 으로 따로).

    로그 기준(bag − 줄 = 1)과 기록기 대응(2)이 다르면 큰 값을 주 열로 쓰고 불일치를 표시한다.
    """
    _rtab(tmp_path, 1788, 1787, judged=2)
    _w(tmp_path, 'db_info.txt', 'Optimized graph:  138 poses')
    _w(tmp_path, 'env.txt', 'launch_exit: clean after 3s\n')
    s = replaystat.summarize(str(tmp_path), 'rtabmap3d')
    assert s['input'] == 1788 and s['input_probe'] == 1788.0 and s['probe_short'] == 0
    assert s['processed'] == 1787 and s['missing_log'] == 1 and s['match_missing'] == 2
    assert s['missing'] == 2 and s['missing_disagree'] is True
    assert s['null_odom'] == 1 and s['imu_wait_drops'] == 1
    assert s['missing_stamps'] == [9.5, 30.0]
    assert s['db_optimized_poses'] == 138 and s['launch_exit'] == 'clean after 3s'
    assert s['fatal'] == 0 and s['tail_exempt'] == 0


def test_summarize_rtabmap_agreeing_counts(tmp_path):
    _rtab(tmp_path, 1788, 1787, judged=1)
    s = replaystat.summarize(str(tmp_path), 'rtabmap3d')
    assert s['missing'] == 1 and s['missing_log'] == 1 and s['missing_disagree'] is False


def test_summarize_rtabmap_log_above_match(tmp_path):
    """로그 기준 누락(3)이 기록기 대응(1)보다 크면 로그 기준이 주 열 — 작은 쪽으로 통과시키지 않는다."""
    _rtab(tmp_path, 1788, 1785, judged=1)
    s = replaystat.summarize(str(tmp_path), 'rtabmap3d')
    assert s['missing'] == 3 and s['missing_log'] == 3 and s['match_missing'] == 1
    assert s['missing_disagree'] is True


def test_summarize_rtabmap_probe_short_does_not_hide_drops(tmp_path):
    """
    측정 노드가 2 장을 놓치고 RTAB-Map 도 2 장을 놓친 경우 (리뷰 B1, fail-open 방지).

    probe − 줄 = 0 이지만 분모는 bag 1 788 이므로 누락 2 — "누락 0" 으로 통과하면 안 된다.
    """
    _rtab(tmp_path, 1786, 1786, judged=2)
    s = replaystat.summarize(str(tmp_path), 'rtabmap3d')
    assert s['input'] == 1788 and s['input_probe'] == 1786.0 and s['probe_short'] == 2
    assert s['missing'] == 2 and s['missing_log'] == 2 and s['missing_disagree'] is False


def test_summarize_rtabmap_without_match_is_unmeasured(tmp_path):
    """대응 파일(match)이 없으면 bag 분모가 없다 — 측정 노드 수로 대신 세지 않고 누락은 미측정(None)."""
    _rtab(tmp_path, 1788, 1788, match=False)
    s = replaystat.summarize(str(tmp_path), 'rtabmap3d')
    assert s['processed'] == 1788 and s['input'] is None and s['input_probe'] == 1788.0
    assert s['missing'] is None and s['missing_log'] is None and s['missing_disagree'] is None
    assert s['probe_short'] is None


def test_summarize_liosam_uses_judged_counts(tmp_path):
    """LIO-SAM 누락은 bag 대응 판정 수, 표의 입력도 bag 수 (측정 노드 1 785 는 따로)."""
    _w(tmp_path, 'probe.json', {'input': {'count': 1785.0}})
    _w(tmp_path, 'log_scan.json', {'fatal': {'x': 1}, 'note': {'Waiting for IMU data': 0}})
    _w(tmp_path, 'match_lio_sam_deskew_cloud_deskewed.json',
       {'input': 1788.0, 'matched': 1786, 'missing': 2, 'judged_missing': 0, 'tail_missing': 2,
        'judged_missing_stamps': []})
    s = replaystat.summarize(str(tmp_path), 'liosam_planar')
    assert s['processed'] == 1786 and s['missing'] == 0 and s['tail_missing'] == 2
    assert s['input'] == 1788 and s['input_probe'] == 1785.0 and s['probe_short'] == 3
    assert s['tail_exempt'] == 2 and s['fatal'] == 1 and s['imu_wait_drops'] == 0


def test_summarize_slam_toolbox_is_unmeasured(tmp_path):
    _w(tmp_path, 'probe.json', {'input': {'count': 1788.0}})
    s = replaystat.summarize(str(tmp_path), 'slam_toolbox')
    assert s['processed'] is None and s['missing'] is None and s['input'] is None
    assert s['input_probe'] == 1788.0 and s['probe_short'] is None


def test_manifest_entry_dim_oracle_ply(tmp_path):
    e = replaystat.manifest_entry(str(tmp_path), 'rtabmap2d_noimu', 2)
    assert e['dim'] == '2d' and e['replay'] == 2 and 'ply' not in e and e['oracle'] is False
    _w(tmp_path, 'cloud_world.ply', 'ply')
    e = replaystat.manifest_entry(str(tmp_path), 'liosam_oracle', 0)
    assert e['dim'] == '3d' and e['oracle'] is True and e['ply'].endswith('cloud_world.ply')
