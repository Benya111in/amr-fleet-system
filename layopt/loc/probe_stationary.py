"""
탐침 (선택, 실험 1 다음): 정지 로봇을 오래 두면 LOST 가 나는가, 나면 왜인가.

P5 대기 슬롯 LOST 는 도착 뒤 52–184 s 정지 중에 났고 원인 미확인이다 (prereg §7). 후보 하나 (layopt-loc, 미검증):
kidnap_monitor 는 정지 중 inlier 를 map_pose_at = (마지막 AMCL 갱신) ∘ (그 뒤 odometry/filtered
상대 이동) 으로 잰다.
AMCL 은 정지 중 갱신하지 않으므로, odom EKF 헤딩이 정지 중 흐르면 합성 자세가 돌아가 inlier 가 떨어진다
(map EKF 는 정합기·AMCL 로 보정돼 흐르지 않는 것처럼 보일 수 있다).

시행 목록(e2 형식)의 행마다: e2_runner 와 같은 배치·시드·안정 → probe_s(기본 360 s sim) 동안 정지 기록.
  - odometry/filtered (odom EKF) 헤딩, map EKF·AMCL·gt, kidnap_status(match_ratio), lost,
    kidnap_monitor 로그.
  - LOST 가 나면 그 시각까지의 odom 헤딩 변화와 match_ratio 추이를 남기고 끝낸다
    (강제 LOST 없음, 판정 없음).
e2_container.sh 를 E2_MODULE=layopt.loc.probe_stationary 로 부른다 (같은 스택·도메인).
"""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

from layopt.loc import e2_runner as er
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import MultiThreadedExecutor


class ProbeNode(er.E2Node):
    """E2Node + odometry/filtered(odom EKF) 기록."""

    def __init__(self) -> None:
        """구독 추가."""
        super().__init__()
        self.odom_rows = []
        self.recording = False
        self.create_subscription(Odometry, f'/{er.NS}/odometry/filtered', self.on_odom, 50)

    def on_odom(self, m: Odometry) -> None:
        """odom EKF 자세 (10 Hz 로 솎음)."""
        if not self.recording:
            return
        t = er.stamp(m.header.stamp)
        if self.odom_rows and t - self.odom_rows[-1][0] < 0.1:
            return
        p = m.pose.pose
        self.odom_rows.append((round(t, 3), round(p.position.x, 4), round(p.position.y, 4),
                               round(er.yaw_of(p.orientation), 6)))


def probe(runner: er.Runner, node: ProbeNode, tr, out_dir: Path) -> dict:
    """탐침 하나."""
    rec = er.Recorder()
    res = {'trial_id': tr['trial_id'], 'started': er.utc(),
           'load_start': [round(v, 2) for v in os.getloadavg()]}
    try:
        runner.place_dummies([])
        ok, out = er.gzcli.set_pose(runner.world, er.NS, tr['x'], tr['y'], tr['yaw'])
        if not ok:
            raise er.Anomaly(f'teleport failed: {out[:200]}')
        node.rec = rec
        res['seed'] = node.seed(tr['x'], tr['y'], tr['yaw'])
        runner.place_dummies(tr['neighbors'][:tr['k']])

        def settled():
            s = node.last['status']
            d, a = runner.gt_err()
            return (s is not None and s.get('state') == 'TRACKING' and not s.get('lost')
                    and d < 0.15 and a < math.radians(5.0))
        if not runner.wait_sim(settled, 40.0, 'settle'):
            raise er.Anomaly('settle_timeout')
        node.odom_rows = []
        node.recording = True
        t0 = node.now()
        lost = {'t': None}

        def lost_now():
            lo = node.last['lost']
            if lo is not None and lo[1] and lo[0] >= t0:
                lost['t'] = lo[0]
                return True
            return False
        runner.wait_sim(lost_now, float(tr.get('probe_s', 360.0)), 'probe')
        node.recording = False
        rows = list(node.odom_rows)
        yaw0 = rows[0][3] if rows else None
        drift = [(r[0] - t0, math.degrees(math.atan2(math.sin(r[3] - yaw0),
                                                     math.cos(r[3] - yaw0)))) for r in rows]
        with rec.lock:
            ratios = [(s['t'] - t0, s.get('match_ratio')) for s in rec.status if s['t'] >= t0]
        res.update({'status': 'done', 'lost': lost['t'] is not None,
                    't_lost': None if lost['t'] is None else round(lost['t'] - t0, 2),
                    'probe_s': round(node.now() - t0, 1),
                    'odom_yaw_drift_deg_end': round(drift[-1][1], 4) if drift else None,
                    'odom_yaw_drift_deg_maxabs': round(max(abs(d) for _, d in drift), 4)
                    if drift else None,
                    'match_ratio_min': min((r for _, r in ratios if r is not None), default=None)})
    except er.Anomaly as exc:
        res.update({'status': 'invalid', 'reason': str(exc)})
    finally:
        node.recording = False
        node.rec = None
    d = out_dir / tr['trial_id']
    d.mkdir(parents=True, exist_ok=True)
    (d / 'odom.csv').write_text('t,x,y,yaw\n' + ''.join(
        f'{r[0]},{r[1]},{r[2]},{r[3]}\n' for r in node.odom_rows), encoding='utf-8')
    with rec.lock:
        (d / 'track.csv').write_text('t,src,x,y,yaw,extra\n' + '\n'.join(rec.rows) + '\n',
                                     encoding='utf-8')
        (d / 'status.jsonl').write_text(''.join(json.dumps(s) + '\n' for s in rec.status),
                                        encoding='utf-8')
        (d / 'rosout.jsonl').write_text(''.join(json.dumps(x, ensure_ascii=False) + '\n'
                                                for x in rec.logs), encoding='utf-8')
    res['ended'] = er.utc()
    res['load_end'] = [round(v, 2) for v in os.getloadavg()]
    (d / 'probe.json').write_text(json.dumps({'trial': tr, 'result': res}, ensure_ascii=False,
                                             indent=1), encoding='utf-8')
    with (out_dir / 'probes.jsonl').open('a', encoding='utf-8') as f:
        f.write(json.dumps(res, ensure_ascii=False) + '\n')
    return res


def main(argv=None) -> int:
    """e2_container.sh 와 같은 인자 (쓰지 않는 것은 무시)."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', required=True)
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--world-name', required=True)
    ap.add_argument('--session', required=True)
    ap.add_argument('--per-session', type=int, default=20)
    ap.add_argument('--max-attempts', type=int, default=2)
    ap.add_argument('--ready-timeout', type=float, default=420.0)
    ap.add_argument('--dummy-mode', default='move')
    ap.add_argument('--only', default='')
    ap.add_argument('--lane', type=int, default=0)
    ap.add_argument('--lanes', type=int, default=1)
    ap.add_argument('--spec-id', default='')
    a = ap.parse_args(argv)
    rclpy.init()
    node = ProbeNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    code = er.EXIT_ANOMALY
    try:
        runner = er.Runner(node, a)
        out_dir = Path(a.run_dir) / 'probes'
        out_dir.mkdir(parents=True, exist_ok=True)
        done = set()
        if (out_dir / 'probes.jsonl').exists():
            done = {json.loads(x)['trial_id'] for x in (out_dir / 'probes.jsonl').read_text(
                encoding='utf-8').splitlines() if x.strip() and json.loads(x)['status'] == 'done'}
        runner.wait_ready()
        runner.setup_dummies()
        code = er.EXIT_DONE
        for tr in sorted(runner.spec['trials'], key=lambda r: r['order']):
            if tr['trial_id'] in done:
                continue
            w0 = time.time()
            r = probe(runner, node, tr, out_dir)
            print(f'[probe] {tr["trial_id"]} {r["status"]} lost={r.get("lost")} '
                  f't_lost={r.get("t_lost")} drift_end={r.get("odom_yaw_drift_deg_end")} '
                  f'min_ratio={r.get("match_ratio_min")} wall={time.time() - w0:.0f}', flush=True)
            if r['status'] != 'done':
                code = er.EXIT_ANOMALY
                break
    except er.Anomaly as exc:
        print(f'[probe] anomaly {exc}', flush=True)
    finally:
        sys.stdout.flush()
        os._exit(code)
    return code


if __name__ == '__main__':
    sys.exit(main())
