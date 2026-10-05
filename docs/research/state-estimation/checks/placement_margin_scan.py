"""강제 배치 자리 주변에서 별칭 여백이 converge_margin 을 넘는 자리가 있는지 훑는다 (읽기 전용).

왜: test_20 의 settle 실패(logs/R12a·R12b·E12 "배치 뒤 위치 추정이 안정되지 않음")는 배치 자세의
별칭 여백이 0.034 (< converge_margin 0.05) 라 로봇이 스스로 수렴을 증명할 수 없기 때문이다.
그러면 자리를 조금 옮겨서 끝나는 문제인지, 아니면 알고리즘(이동으로 별칭 가르기, research brief
state-estimation §4.C 의 DISAMBIGUATE)이 필요한 문제인지 가려야 한다. Gazebo 불필요, 수 초.

    python3 docs/research/state-estimation/checks/placement_margin_scan.py

**결론 (2026-10-05 실측, placement_margin_scan_out.txt)**: victim 자리 (0.8, 5) 주변 ±1 m ·
4 방위 100 곳 중 임계를 넘는 곳은 **8 곳뿐**이고 전부 x = 1.80 (동쪽 1 m) 이다. 가장 높은 곳도
0.063 으로 임계 대비 여유가 0.013 밖에 없다. 따라서 **자리 이동은 믿을 만한 해법이 아니다**:
여유가 스캔 잡음 수준이고, 게다가 그 자리는 "구역 서쪽 입구 1.2 m 앞"(test_12…:62) 이라는
교착 기하 조건과 충돌한다 (1 m 동쪽이면 입구 0.2 m 앞이 된다). 정공법은 §4.C 다 —
정지한 360° LiDAR 로 가를 수 없는 것을 **이동**이 가른다.

이 스크립트는 "그 자리에서 LiDAR 만으로 가를 수 있는가" 만 답한다. 실제 실패 여부는 시드
타이밍·스캔 잡음·신뢰 창(external_trust_window) 이 닫혔는지에도 달려 있다 — 여백이 작다는 것이
항상 실패를 뜻하지는 않는다.
"""
import math
import sys
from pathlib import Path

REPO = Path('/ros2_ws')
sys.path.insert(0, str(REPO / 'docs/research/state-estimation/checks'))
sys.path.insert(0, str(REPO / 'src/amr_localization'))

import alias_margin as am          # noqa: E402  (load_grid/raycast/상수를 재사용)
from amr_localization import global_seed                                  # noqa: E402
from amr_localization.scan_map_match import DistanceField, match_ratio    # noqa: E402

occ, spec = am.load_grid()
field = DistanceField(occ.ravel().tolist(), spec, 65)
field._occ = occ

CONVERGE = 0.05


def margin_at(pose, seed=0):
    ranges, amin, inc = am.raycast(field._occ, field.spec, pose, noise=0.03, seed=seed)
    hyps = global_seed.search(field, field.free, ranges, amin, inc, am.RANGE_MAX, am.LIDAR)
    beams = global_seed.base_beams(ranges, amin, inc, am.RANGE_MAX, 180, am.LIDAR)
    alts = global_seed.alternative_poses(hyps, pose, (0.0, 0.0, 0.0))
    m = global_seed.alias_margin(field, beams, pose, alts, 0.2)
    rho, n = match_ratio(field, pose, am.LIDAR, ranges, amin, inc, am.RANGE_MAX, 0.2, 180)
    return m, rho, n


print('강제 배치 자리 주변 훑기 (victim 기준 (0.8, 5, 0) — test_12…:62)')
print(f'{"자세":>22}  {"여백":>7}  {"rho":>5}  판정')
best = []
for dx in (-1.0, -0.5, 0.0, 0.5, 1.0):
    for dy in (-1.0, -0.5, 0.0, 0.5, 1.0):
        for yaw in (0.0, math.pi / 2, math.pi, -math.pi / 2):
            p = (0.8 + dx, 5.0 + dy, yaw)
            try:
                m, rho, n = margin_at(p)
            except Exception as exc:
                print(f'({p[0]:6.2f},{p[1]:6.2f},{math.degrees(p[2]):4.0f}d)  오류 {exc!r}')
                continue
            if m is None or n < 30:
                continue
            ok = '통과' if m >= CONVERGE else ''
            best.append((m, p, rho))
            if m >= CONVERGE or (dx == 0.0 and dy == 0.0):
                print(f'({p[0]:6.2f},{p[1]:6.2f},{math.degrees(p[2]):4.0f}d)  '
                      f'{m:7.3f}  {rho:5.3f}  {ok}')
best.sort(reverse=True)
print()
print(f'훑은 자리 {len(best)} 곳 · 여백 ≥ {CONVERGE} 인 곳 {sum(1 for m, _, _ in best if m >= CONVERGE)} 곳')
print('가장 높은 5 곳:')
for m, p, rho in best[:5]:
    print(f'   ({p[0]:6.2f},{p[1]:6.2f},{math.degrees(p[2]):4.0f}d)  여백 {m:.3f}  rho {rho:.3f}')
