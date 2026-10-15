"""E3(b) 배포 2D 사슬 kidnap_monitor LOST 셈 (loc_lost.py) 시험."""

import importlib.util
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location('loc_lost', PKG / 'scripts' / 'loc_lost.py')
ll = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ll)


def test_lost_counts_transitions():
    assert ll.lost_counts([]) == (0, 0, 0)
    assert ll.lost_counts([False, False]) == (2, 0, 0)
    # 처음이 True 면 사건 1, 끊겼다 다시 True 면 사건 2
    assert ll.lost_counts([True, True, False, True]) == (4, 3, 2)
    assert ll.lost_counts([False, True, False, False, True, True]) == (6, 3, 2)


def test_summarize_missing_out_bag_is_unmeasured(tmp_path):
    r = ll.summarize(str(tmp_path))
    assert r['lost_msgs'] is None and r['lost_events'] is None
    assert r['amcl_pose_msgs'] is None
