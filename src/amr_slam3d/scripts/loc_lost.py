#!/usr/bin/env python3
"""
E3(b) 2D 배포 사슬의 kidnap_monitor LOST 세기 (사전 등록 항목 `2026-10-09_0047_b2-eval_loc-ablation-prereg.md`).

사전 등록 문구: "2D 의 kidnap_monitor LOST 는 out_bag 의 `localization/lost` 로 따로 센다". 이 스크립트가 그 셈이다
(E3(b) 재생 결과를 보기 전에 만들었다 — 지표 정의는 사전 등록 그대로, 도구만 새로).

    loc_lost.py --root R --stage e3 --out R/analysis/e3/lost.jsonl     # 컨테이너(rosbag2_py) 안

재생 하나(amcl_* 행)마다: `localization/lost` 메시지 수, data=True 수, False→True 전이 수(첫 메시지가 True 면 1 로 센다),
`localization/kidnap_status` 수와 마지막 상태 JSON 의 `state`(있으면), `amcl_pose` 수. out_bag 에 토픽이 없으면 None
(미측정 — 0 으로 쓰지 않는다).
"""

import argparse
import json
import os
import re
import sys

LOST = '/amr_01/localization/lost'
STATUS = '/amr_01/localization/kidnap_status'
AMCL = '/amr_01/amcl_pose'


def lost_counts(values):
    """Bool 값 열 → (메시지 수, True 수, False→True 전이 수 — 처음이 True 면 1)."""
    n = len(values)
    n_true = sum(1 for v in values if v)
    trans = 0
    prev = False
    for v in values:
        if v and not prev:
            trans += 1
        prev = bool(v)
    return n, n_true, trans


def _read(bag, topics):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import bag_tools
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader, types = bag_tools._reader(bag)
    have = [t for t in topics if t in types]
    out = {t: None for t in topics}
    if not have:
        return out
    for t in have:
        out[t] = []
    reader.set_filter(rosbag2_py.StorageFilter(topics=have))
    cls = {t: get_message(types[t]) for t in have}
    while reader.has_next():
        t, raw, _ = reader.read_next()
        out[t].append(deserialize_message(raw, cls[t]))
    return out


def summarize(d):
    """재생 디렉터리 하나 → dict (out_bag 이 없으면 None 들)."""
    bag = os.path.join(d, 'out_bag')
    res = {'lost_msgs': None, 'lost_true': None, 'lost_events': None, 'status_msgs': None,
           'status_last_state': None, 'amcl_pose_msgs': None}
    if not os.path.isdir(bag):
        return res
    m = _read(bag, (LOST, STATUS, AMCL))
    if m[LOST] is not None:
        res['lost_msgs'], res['lost_true'], res['lost_events'] = lost_counts(
            [bool(x.data) for x in m[LOST]])
    if m[STATUS] is not None:
        res['status_msgs'] = len(m[STATUS])
        if m[STATUS]:
            try:
                res['status_last_state'] = json.loads(m[STATUS][-1].data).get('state')
            except (ValueError, AttributeError):
                res['status_last_state'] = None
    if m[AMCL] is not None:
        res['amcl_pose_msgs'] = len(m[AMCL])
    return res


def main(argv=None):
    """명령줄."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--root', required=True)
    ap.add_argument('--stage', default='e3')
    ap.add_argument('--out', required=True)
    ns = ap.parse_args(argv)
    base = os.path.join(ns.root, ns.stage)
    os.makedirs(os.path.dirname(os.path.abspath(ns.out)), exist_ok=True)
    n = 0
    with open(ns.out, 'w', encoding='utf-8') as f:
        for bag in sorted(os.listdir(base)):
            for row in sorted(os.listdir(os.path.join(base, bag))):
                if not row.startswith('amcl_'):
                    continue
                rd = os.path.join(base, bag, row)
                for rk in sorted(os.listdir(rd)):
                    mm = re.fullmatch(r'r(\d+)', rk)
                    if not mm or not os.path.isfile(os.path.join(rd, rk, 'DONE')):
                        continue
                    r = {'bag': bag, 'row': row, 'replay': int(mm.group(1))}
                    r.update(summarize(os.path.join(rd, rk)))
                    f.write(json.dumps(r, ensure_ascii=False) + '\n')
                    n += 1
    print(f'{n} 재생 → {ns.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
