"""좁은 통로 측위 별칭의 오프라인 재현 (읽기 전용, Gazebo 불필요, 수 초).

maps/warehouse.pgm 을 map_server 와 같은 규칙으로 읽고, 같은 격자 위에서 720 빔 2D 스캔을
광선투사로 합성한 뒤, kidnap_monitor_node 가 쓰는 것과 **같은** amr_localization.global_seed
.search / .alias_margin 을 돌린다.

쓰는 좌표는 지어낸 것이 아니라 시험 코드에서 그대로 가져왔다:
  test_12_multi_robot_deadlock.py:61-62  FORCED_BLOCKER (3,6,0) · FORCED_VICTIM (0.8,5,0)
  test_05_kidnapped_robot.py:32-33       TARGETS 5 개

왜 이것이 필요한가: 시나리오 12 를 1 회 돌리는 데 62~67 분이 들고 그 안에 배치는 2 회뿐이라
별칭 빈도를 통계로 판정할 수 없다. 이 스크립트는 같은 질문(그 자리에서 LiDAR 만으로 위치를
가를 수 있는가)을 결정적으로, 수 초에 답한다.

    python3 docs/research/state-estimation/checks/alias_margin.py

의존: numpy, scipy (amr_localization.scan_map_match 가 scipy.ndimage 를 쓴다).
"""
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[4]      # docs/research/state-estimation/checks/ -> 저장소 루트
sys.path.insert(0, str(REPO / 'src' / 'amr_localization'))
from amr_localization import global_seed                     # noqa: E402
from amr_localization.scan_map_match import DistanceField, GridSpec, match_ratio  # noqa: E402

RES = 0.05
OX, OY = -30.1783, -20.1938
OCC_T, FREE_T = 0.65, 0.19
LIDAR = (0.15, 0.0, 0.0)
RANGE_MAX = 25.0
N_BEAMS = 720


def load_grid():
    data = (REPO / 'maps' / 'warehouse.pgm').read_bytes()
    assert data[:2] == b'P5'
    tok, i = [], 2
    while len(tok) < 3:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tok.append(int(data[i:j]))
        i = j
    i += 1
    w, h, _ = tok
    img = np.frombuffer(data[i:i + w * h], np.uint8).reshape(h, w)
    img = img[::-1]                       # pgm row 0 = max y  ->  OccupancyGrid row 0 = min y
    p = (255.0 - img.astype(np.float64)) / 255.0            # map_server trinary occupancy
    occ = np.full((h, w), -1, np.int16)
    occ[p > OCC_T] = 100
    occ[p < FREE_T] = 0
    return occ, GridSpec(w, h, RES, OX, OY, 0.0)


def raycast(occ, spec, pose, n=N_BEAMS, rmax=RANGE_MAX, noise=0.03, seed=0):
    """360 deg scan from the LiDAR origin of base pose `pose` (angle_min = -pi)."""
    rng = np.random.default_rng(seed)
    bx, by, byaw = pose
    lx = bx + math.cos(byaw) * LIDAR[0] - math.sin(byaw) * LIDAR[1]
    ly = by + math.sin(byaw) * LIDAR[0] + math.cos(byaw) * LIDAR[1]
    lyaw = byaw + LIDAR[2]
    amin, inc = -math.pi, 2.0 * math.pi / n
    occupied = occ >= 65
    step = RES / 3.0
    out = []
    for k in range(n):
        a = lyaw + amin + k * inc
        dx, dy = math.cos(a), math.sin(a)
        r, hit = 0.0, math.inf
        while r < rmax:
            c = int(math.floor((lx + r * dx - spec.origin_x) / RES))
            rr = int(math.floor((ly + r * dy - spec.origin_y) / RES))
            if not (0 <= rr < spec.height and 0 <= c < spec.width):
                break
            if occupied[rr, c]:
                hit = r
                break
            r += step
        if math.isfinite(hit) and noise:
            hit += rng.normal(0.0, noise)
        out.append(hit)
    return out, amin, inc


def report(field, pose, label, seed=0, noise=0.03):
    ranges, amin, inc = raycast(field._occ, field.spec, pose, noise=noise, seed=seed)
    hyps = global_seed.search(field, field.free, ranges, amin, inc, RANGE_MAX, LIDAR)
    beams = global_seed.base_beams(ranges, amin, inc, RANGE_MAX, 180, LIDAR)
    alts = global_seed.alternative_poses(hyps, pose, (0.0, 0.0, 0.0))
    margin = global_seed.alias_margin(field, beams, pose, alts, 0.2)
    rho, valid = match_ratio(field, pose, LIDAR, ranges, amin, inc, RANGE_MAX, 0.2, 180)
    print(f'{label}  truth=({pose[0]:.2f},{pose[1]:.2f},{math.degrees(pose[2]):.0f}d) '
          f'rho={rho:.3f} n={valid}')
    for k, h in enumerate(hyps, 1):
        d = math.hypot(h.x - pose[0], h.y - pose[1])
        print(f'   #{k} ({h.x:7.2f},{h.y:7.2f},{math.degrees(h.yaw):6.0f}d) '
              f'score={h.score:.4f} ratio={h.ratio:.3f}  d_truth={d:5.2f} m')
    gap = (hyps[0].ratio - hyps[1].ratio) if len(hyps) > 1 else float('nan')
    print(f'   top1-top2 ratio gap = {gap:.3f}   alias_margin(refined) = '
          f'{"None" if margin is None else f"{margin:.3f}"}   '
          f'[converge_margin 0.05, alias_reject_margin 0.15]')
    return hyps, gap, margin


CONVERGE_MARGIN = 0.05     # src/amr_localization/config/kidnap_monitor.yaml
ALIAS_REJECT_MARGIN = 0.15  # kidnap_monitor_node.py 기본값

occ, spec = load_grid()
field = DistanceField(occ.ravel().tolist(), spec, 65)
field._occ = occ
print('map', spec.width, 'x', spec.height, '@', spec.resolution, 'free cells', int(field.free.sum()))
print()
# test_12 forced placement
report(field, (3.0, 6.0, 0.0), 'test_12 blocker amr_05 (3,6,0)')
print()
report(field, (0.8, 5.0, 0.0), 'test_12 victim  amr_04 (0.8,5,0)')
print()
# scenario 05 targets
for t in [(12.0, 6.0, math.pi / 2), (-12.0, -6.0, 0.0), (21.0, 0.0, math.pi),
          (-21.0, 13.0, -math.pi / 2), (0.0, 0.0, 0.0)]:
    report(field, t, 'scenario 05 target')
    print()

# ---------------------------------------------------------------- 요약
print('=' * 72)
print('요약 — 통로가 6 m 주기라 ±6 m · ±12 m 지점이 거의 같은 점수를 낸다.')
print(f'  converge_margin = {CONVERGE_MARGIN} (이보다 여백이 작으면 수렴으로 보지 않는다)')
print()
print('  test_12 배치점은 시나리오 05 목표점보다 여백이 작다 — 그래서 05 는 5/5 통과하는데')
print('  test_20 은 산발적으로 settle 에 실패한다. 특히 피해 로봇 자리는 임계값 아래다.')
print()
print('  이 스크립트는 "그 자리에서 LiDAR 만으로 가를 수 있는가" 만 답한다. 실제 실패 여부는')
print('  시드 타이밍·스캔 잡음에도 달려 있으므로, 여백이 작다 == 항상 실패 는 아니다.')
