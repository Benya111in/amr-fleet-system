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


def test_summarize_rtabmap_counts_log_lines(tmp_path):
    """RTAB-Map 누락 = 입력 − 로그 처리 줄 수 (ratio=0 줄은 처리로 세되 null_odom 으로 따로)."""
    _w(tmp_path, 'probe.json', {'input': {'count': 1788.0}, 'traj_online': {'n': 1786},
                                'tf': {'valid': True}})
    _w(tmp_path, 'log_scan.json', {'fatal': {'a': 0, 'b': 0},
                                   'note': {"didn't receive IMU newer": 1,
                                            'Odom: ratio=0.000000': 1},
                                   'common': {}, 'processed_log_lines': 1787})
    _w(tmp_path, 'match_amr_01_icp_odom.json', {'missing': 2, 'judged_missing': 2,
                                                'judged_missing_stamps': [9.5, 30.0]})
    _w(tmp_path, 'db_info.txt', 'Optimized graph:  138 poses')
    _w(tmp_path, 'env.txt', 'launch_exit: clean after 3s\n')
    s = replaystat.summarize(str(tmp_path), 'rtabmap3d')
    assert s['processed'] == 1787 and s['missing'] == 1
    assert s['null_odom'] == 1 and s['imu_wait_drops'] == 1
    assert s['match_missing'] == 2 and s['missing_stamps'] == [9.5, 30.0]
    assert s['db_optimized_poses'] == 138 and s['launch_exit'] == 'clean after 3s'
    assert s['fatal'] == 0 and s['tail_exempt'] == 0


def test_summarize_liosam_uses_judged_counts(tmp_path):
    _w(tmp_path, 'probe.json', {'input': {'count': 1788.0}})
    _w(tmp_path, 'log_scan.json', {'fatal': {'x': 1}, 'note': {'Waiting for IMU data': 0}})
    _w(tmp_path, 'match_lio_sam_deskew_cloud_deskewed.json',
       {'matched': 1786, 'missing': 2, 'judged_missing': 0, 'tail_missing': 2,
        'judged_missing_stamps': []})
    s = replaystat.summarize(str(tmp_path), 'liosam_planar')
    assert s['processed'] == 1786 and s['missing'] == 0 and s['tail_missing'] == 2
    assert s['tail_exempt'] == 2 and s['fatal'] == 1 and s['imu_wait_drops'] == 0


def test_summarize_slam_toolbox_is_unmeasured(tmp_path):
    _w(tmp_path, 'probe.json', {'input': {'count': 1788.0}})
    s = replaystat.summarize(str(tmp_path), 'slam_toolbox')
    assert s['processed'] is None and s['missing'] is None and s['input'] == 1788.0


def test_manifest_entry_dim_oracle_ply(tmp_path):
    e = replaystat.manifest_entry(str(tmp_path), 'rtabmap2d_noimu', 2)
    assert e['dim'] == '2d' and e['replay'] == 2 and 'ply' not in e and e['oracle'] is False
    _w(tmp_path, 'cloud_world.ply', 'ply')
    e = replaystat.manifest_entry(str(tmp_path), 'liosam_oracle', 0)
    assert e['dim'] == '3d' and e['oracle'] is True and e['ply'].endswith('cloud_world.ply')
