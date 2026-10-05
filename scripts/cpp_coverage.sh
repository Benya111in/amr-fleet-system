#!/usr/bin/env bash
# C++ 줄 커버리지 (명세 4장 10절 "주요 모듈의 테스트 커버리지 70 % 이상").
#
# Python 은 colcon coveragepy-result 로 이미 재고 있지만, 명세가 말하는 "주요 모듈"
# (DWA · A* · EKF · BT) 은 전부 C++ 이라 그것만으로는 충족을 주장할 수 없다.
#
# 사용: ./scripts/cpp_coverage.sh [패키지...]
#   기본은 C++ 코어가 있는 네 패키지. 별도 빌드 디렉터리를 써서 배포 빌드를 건드리지 않는다.
set -euo pipefail

PKGS=${*:-"amr_navigation amr_perception amr_localization amr_behavior"}
BUILD=build_cov
INSTALL=install_cov
OUT=logs/coverage

# 출처를 남긴다 — 이게 없어서 logs/coverage 가 어느 커밋 것인지 못 따졌다 (전수 감사 지적).
# 통합 시나리오는 run_integration.sh 가 HEAD.txt·uncommitted.txt 를 남기는데 커버리지는 빠져 있었다.
_snap_provenance() {
    local out="$1"
    mkdir -p "$out"
    # git 워크트리를 컨테이너에 마운트하면 .git 이 마운트 **밖**(/home/<user>/.git/worktrees/...)을
    # 가리켜 컨테이너 안에서는 못 읽는다 (확인: "fatal: not a git repository").
    # 그래서 호스트가 AMR_HEAD 로 넘겨주면 그것을 쓰고, 없으면 컨테이너 안에서 시도하고,
    # 그마저 안 되면 **모른다고 적는다** — 빈 값이나 거짓 값을 남기지 않는다.
    if [ -n "${AMR_HEAD:-}" ]; then
        printf '%s\n' "${AMR_HEAD}" > "$out/HEAD.txt"
        printf '%s\n' "${AMR_UNCOMMITTED:-}" > "$out/uncommitted.txt"
    elif git rev-parse --short HEAD > "$out/HEAD.txt" 2>/dev/null; then
        git status --short > "$out/uncommitted.txt" 2>/dev/null || true
    else
        printf 'UNKNOWN (컨테이너에서 git 접근 불가 — 호스트에서 AMR_HEAD 로 넘겨라)\n' \
            > "$out/HEAD.txt"
        : > "$out/uncommitted.txt"
    fi
    date -Is > "$out/measured_at.txt"
}

echo "== 커버리지 빌드 ($PKGS) =="
# --coverage 는 계측 + libgcov 링크. -O0 이어야 줄 대응이 정확하다.
colcon build --packages-select $PKGS \
  --build-base "$BUILD" --install-base "$INSTALL" \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug \
               -DCMAKE_CXX_FLAGS="--coverage -O0 -g" \
               -DCMAKE_EXE_LINKER_FLAGS="--coverage" \
               -DCMAKE_SHARED_LINKER_FLAGS="--coverage" \
  2>&1 | tail -3

echo "== 기준선(0 카운트) 캡처 =="
mkdir -p "$OUT"
_snap_provenance "$OUT"
lcov --capture --initial --directory "$BUILD" --output-file "$OUT/base.info" \
  --rc lcov_branch_coverage=0 >/dev/null 2>&1

echo "== 단위 시험 실행 =="
# 통합 시험(gazebo)은 제외한다 — 여기서 재는 것은 단위 커버리지다.
colcon test --packages-select $PKGS --build-base "$BUILD" --install-base "$INSTALL" \
  --ctest-args -LE "linter" 2>&1 | tail -3 || true

echo "== 캡처·병합 =="
lcov --capture --directory "$BUILD" --output-file "$OUT/test.info" \
  --rc lcov_branch_coverage=0 >/dev/null 2>&1
lcov --add-tracefile "$OUT/base.info" --add-tracefile "$OUT/test.info" \
  --output-file "$OUT/all.info" --rc lcov_branch_coverage=0 >/dev/null 2>&1

# 우리 소스만 남긴다 (시스템 헤더·gtest·시험 코드 제외).
lcov --extract "$OUT/all.info" "*/src/amr_*/src/*" "*/src/amr_*/include/*" \
  --output-file "$OUT/src.info" --rc lcov_branch_coverage=0 >/dev/null 2>&1
lcov --remove "$OUT/src.info" "*/test/*" "*/tools/*" \
  --output-file "$OUT/final.info" --rc lcov_branch_coverage=0 >/dev/null 2>&1

echo
echo "== 전체 =="
lcov --summary "$OUT/final.info" --rc lcov_branch_coverage=0 2>&1 | grep -E "lines|functions"

echo
echo "== 명세가 말하는 '주요 모듈' (DWA · A* · EKF · BT) =="
python3 - "$OUT/final.info" <<'PY'
import re, sys, collections
key = {'dwa': 'DWA', 'astar': 'A*', 'ekf': 'EKF', 'kalman': 'EKF(칼만)',
       'behavior_tree': 'BT', 'task_tree': 'BT', 'bt_': 'BT'}
cur = None
hit = collections.defaultdict(int); tot = collections.defaultdict(int)
fhit = 0; ftot = 0
per = {}
for line in open(sys.argv[1]):
    if line.startswith('SF:'):
        cur = line[3:].strip(); per.setdefault(cur, [0, 0])
    elif line.startswith('DA:') and cur:
        _, rest = line.split(':', 1)
        _, cnt = rest.strip().split(',')[:2]
        per[cur][1] += 1
        if int(cnt) > 0:
            per[cur][0] += 1
for path, (h, t) in sorted(per.items()):
    base = path.split('/')[-1].lower()
    for k, label in key.items():
        if k in base:
            hit[label] += h; tot[label] += t
            break
for label in sorted(tot):
    h, t = hit[label], tot[label]
    mark = 'OK ' if t and 100 * h / t >= 70 else '미달'
    print(f'  {mark} {label:10s} {h:6d}/{t:<6d} = {100*h/max(t,1):5.1f} %')
print()
print('  파일별 상위 미달 (줄 수 50 이상):')
low = [(100*h/t, p, h, t) for p, (h, t) in per.items() if t >= 50 and 100*h/t < 70]
for pct, p, h, t in sorted(low)[:12]:
    print(f'    {pct:5.1f} %  {h:5d}/{t:<5d}  {"/".join(p.split("/")[-2:])}')
PY
echo
echo "출력: $OUT/final.info  (genhtml $OUT/final.info -o $OUT/html 로 HTML)"
