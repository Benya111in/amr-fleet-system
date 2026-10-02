#!/usr/bin/env bash
# scripts/check_commit_msg.sh 의 수용/거부 표를 고정한다.
#
#   bash scripts/test_check_commit_msg.sh        # 로컬
#   CI: commit-lint job 의 한 단계 (ROS 불필요, 1초 이내)
#
# 왜 있는가: commit-lint job 자체는 저장소에 이미 있는(= 통과하는) 메시지만 먹인다. 규칙을
# 느슨하게 만드는 회귀 — 예를 들어 scope 문자 클래스에 ',' 를 넣으면 '(a,,b)' 'feat(a,)' 까지
# 통과해 버리는 것 — 를 잡는 검사가 그 job 에는 없다. 이 표가 그 자리를 메운다.
# 규칙을 바꿀 때는 이 표를 먼저 고친다.
#
# 종료 코드: 0 전부 기대대로, 1 하나라도 어긋남
set -u

HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
CHECK="${HERE}/check_commit_msg.sh"
[ -r "$CHECK" ] || { echo "검사 대상이 없다: $CHECK" >&2; exit 1; }

pass=0
fail=0

# 기대 종료 코드대로인지 본다. stdout/stderr 는 버린다 (경고는 판정에 쓰지 않는다).
expect() {
    local want=$1 subject=$2 note=${3:-}
    local got=0
    printf '%s\n' "$subject" | bash "$CHECK" >/dev/null 2>&1 || got=$?
    if [ "$got" = "$want" ]; then
        pass=$((pass + 1))
    else
        fail=$((fail + 1))
        printf '  어긋남: rc=%s (기대 %s) %s\n    첫 줄: %s\n' \
            "$got" "$want" "${note:+— $note}" "$subject" >&2
    fi
}
ok()     { expect 0 "$1" "${2:-}"; }   # 통과해야 한다
reject() { expect 1 "$1" "${2:-}"; }   # 형식 위반으로 거부해야 한다

echo "--- 통과해야 하는 것"

# 기본 형식
ok 'feat: 새 기능'
ok 'fix(localization): EKF 공분산 초기화 오류 수정'
ok 'feat(navigation)!: 플래너 인터페이스 변경'
ok 'chore: 잡무'

# type 전체 — TYPES 목록과 이 줄들이 함께 움직여야 한다
for t in feat fix docs style refactor perf tune test build ci chore revert; do
    ok "${t}: 제목" "type ${t}"
done

# scope 문자: 소문자/숫자/- _ . /
ok 'feat(a-b_c.d/e): 제목' 'scope 허용 문자'
ok 'feat(nav2): 제목' 'scope 에 숫자'
ok 'feat(a): 제목' 'scope 한 글자'

# 쉼표 scope — 이번에 허용한 것
ok 'feat(itest,scripts): ETA 15 % 를 07 에서 판정한다' '쉼표 scope 2개'
ok 'feat(a,b,c): 제목' '쉼표 scope 3개'
ok 'feat(a-b,c.d): 제목' '쉼표 항목에도 허용 문자'
ok 'feat(itest,scripts)!: 제목' '쉼표 scope + breaking'

# git/GitHub 자동 생성 메시지
ok "Merge branch 'feature/x' into develop"
ok 'Merge pull request #12 from Benya111in/feature/x'
ok 'Revert "feat(navigation): A* 플래너"'
ok 'fixup! feat: 제목'
ok 'squash! feat: 제목'

# 길이 경계: 72자 정확히는 통과, 73자는 거부(아래)
b72="feat: $(printf 'ㄱ%.0s' $(seq 1 66))"
ok "$b72" '72자 정확히 (UTF-8 문자 수 기준)'

# 이번 PR 범위에서 실제로 걸렸던 8건 — 전부 통과해야 한다
echo "--- 히스토리에 있던 8건 (규칙 완화 대상)"
ok 'tune(behavior): 마커 모호성 임계 0.05 → 0.02 rad² (σ 12.8° → 8.1°)'
ok 'tune(behavior): 마커 모호성 임계 0.01 → 0.05 rad²'
ok 'tune(navigation): 통로 이탈 후진 속도 0.25 → 0.5 m/s'
ok 'tune(navigation): 횡단 통로 반폭 여유 0.5 → 0.7 m'
ok 'tune(navigation): 횡단 통로에 덜 들어가도록 여유를 키운다 (1.0 → 2.0 s)'
ok 'feat(itest,scripts): ETA 15 % 를 07 에서 판정한다 + 커버리지 산출물에 출처를 남긴다'
ok 'feat(behavior,bringup,docs): 명세 조항 3건 충족 — 환경 검증 산출물 · RViz2 설정 · BT 모듈성'
ok 'feat(docs,fleet,nav): 명세 조항 3건 충족 — colcon 경고 0 · TF 시각화 · 할당 최적성 분석'

echo "--- 거부해야 하는 것 (규칙이 느슨해지지 않았는지)"

# 쉼표를 허용했기 때문에 새로 생긴 구멍들 — 여기가 이 파일의 존재 이유다
reject 'feat(,): 제목'      '빈 scope'
reject 'feat(a,): 제목'     '쉼표 뒤가 빈 항목'
reject 'feat(,a): 제목'     '쉼표 앞이 빈 항목'
reject 'feat(a,,b): 제목'   '가운데 빈 항목'
reject 'feat(a,-b): 제목'   '쉼표 항목이 - 로 시작'
reject 'feat(a,.b): 제목'   '쉼표 항목이 . 로 시작'
reject 'feat(a,_b): 제목'   '쉼표 항목이 _ 로 시작'
reject 'feat(a,/b): 제목'   '쉼표 항목이 / 로 시작'
reject 'feat(a, b): 제목'   '쉼표 뒤 공백'
reject 'feat(a,B): 제목'    '쉼표 항목에 대문자'
# 위 네 글자(- . _ /)는 사후 검사 루프의 `[a-z0-9]*` 글로브 하나가 책임진다. 넷을 다 적어야
# 그 글로브를 [a-z0-9_/]* 로 넓히는 회귀가 잡힌다 — - 와 . 만 있으면 그 변이가 표를 통과한다.

# type 경계 — 'tune' 추가가 접두어 일치로 새 구멍을 만들지 않았는지
reject 'tuned: 제목'        'tune 으로 시작하는 다른 낱말'
reject 'tuning(nav): 제목'  'tune 접두어 + 다른 글자'
reject 'tun: 제목'          'tune 의 앞부분'
reject 'TUNE: 제목'         '대문자 type'
reject 'Tune: 제목'         '첫 글자만 대문자'
reject 'wip: 제목'          '목록에 없는 type'
reject 'feature: 제목'      'feat 접두어 + 다른 글자'

# type **앞**에 글자가 붙는 방향 = 정규식의 '^' 앵커. 위 줄들은 모두 type 뒤만 보므로
# '^' 를 지우는 회귀를 잡지 못한다 (그 변이는 아래 네 줄이 없으면 표를 그대로 통과한다).
reject 'Update docs: 설명 보강'    'type 앞에 낱말 (앵커)'
reject 'xx feat: 제목'             'type 앞에 군더더기'
reject '  feat: 제목'              '선행 공백 (앞 공백은 깎지 않는다)'
reject '리뷰 반영 tune: 임계 조정'  'type 앞에 문장'

# '!' 는 하나만. (!)? 를 (!*) 로 넓히는 회귀를 잡는다.
reject 'feat!!: 제목'       "'!' 둘"
reject 'feat(a)!!: 제목'    "scope 뒤 '!' 둘"
reject 'feat!(a): 제목'     "'!' 가 scope 앞"

# 기존 규칙이 그대로인지
reject 'feat(A): 제목'      'scope 대문자'
reject 'feat(-a): 제목'     'scope 가 - 로 시작'
reject 'feat(.a): 제목'     'scope 가 . 로 시작'
reject 'feat(a b): 제목'    'scope 에 공백'
reject 'feat(a): '          'subject 가 비어 있다'
reject 'feat: '             'scope 없이 subject 가 비어 있다'
reject 'feat:제목'          "':' 뒤 공백 없음"
reject 'feat:  제목'        "':' 뒤 공백 둘"
reject '제목만 있다'          "'<type>: ' 접두어 없음"
reject ''                   '빈 메시지'
reject 'feat(a,b)x: 제목'   '괄호 뒤에 군더더기'
reject "feat: $(printf 'ㄱ%.0s' $(seq 1 67))" '73자 — 상한 초과'

echo "--- 첫 유효 줄 추출 (여러 줄 · CRLF · 공백 · 주석 · 두 호출 경로)"
# 위의 ok()/reject() 는 `printf '%s\n'` 로 **한 줄만** 먹인다. 그래서 빈 줄/주석 skip, CR 제거,
# 줄 끝 공백 제거가 표에서 한 번도 실행되지 않았다 — 그 고리를 망가뜨리는 변이가 69건을 그대로
# 통과했다. 아래는 원문을 그대로 먹여 그 고리를 고정한다. '%b' 라서 \n \r 이 escape 로 해석된다.

# stdin 경로 = CI 의 `git show -s --format=%B <sha> | check` 와 PR 제목
expect_raw() {
    local want=$1 raw=$2 note=${3:-}
    local got=0
    printf '%b' "$raw" | bash "$CHECK" >/dev/null 2>&1 || got=$?
    if [ "$got" = "$want" ]; then
        pass=$((pass + 1))
    else
        fail=$((fail + 1))
        printf '  어긋남(stdin): rc=%s (기대 %s) — %s\n' "$got" "$want" "$note" >&2
    fi
}
# 파일 인자 경로 = setup_gitflow.sh 가 심은 commit-msg 훅 (`bash check_commit_msg.sh "$1"`)
expect_file() {
    local want=$1 raw=$2 note=${3:-}
    local f got=0
    f=$(mktemp) || { fail=$((fail + 1)); echo "  mktemp 실패" >&2; return; }
    printf '%b' "$raw" > "$f"
    bash "$CHECK" "$f" >/dev/null 2>&1 || got=$?
    rm -f "$f"
    if [ "$got" = "$want" ]; then
        pass=$((pass + 1))
    else
        fail=$((fail + 1))
        printf '  어긋남(파일): rc=%s (기대 %s) — %s\n' "$got" "$want" "$note" >&2
    fi
}

# 빈 줄 · CRLF · 본문
expect_raw 0 'feat(a): 제목\n\n본문 한 줄\n'        '여러 줄 — 첫 줄로 판정'
expect_raw 0 '\n\nfeat(a): 제목\n'                  '선행 빈 줄 건너뛰기'
expect_raw 0 'feat(a): 제목\r\n본문\r\n'            'CRLF — 짧은 줄'
# CR 제거를 고정하려면 CR 이 **판정을 바꾸는** 입력이어야 한다. 짧은 줄은 CR 이 남아도
# 정규식의 (.*)$ 가 먹고 길이도 상한 아래라 통과해 버린다 (그 변이가 표를 그대로 통과했다).
expect_raw 0 "${b72}\r\n본문\r\n"                   'CRLF — 72자 + CR: CR 을 세면 73자가 된다'
expect_raw 0 '\r\nfeat(a): 제목\r\n'                'CR 만인 첫 줄은 빈 줄로 본다'
expect_raw 0 '   \nfeat(a): 제목\n'                 '공백만인 첫 줄은 빈 줄로 본다'
expect_raw 0 '\t\nfeat(a): 제목\n'                  '탭만인 첫 줄도 빈 줄'
# 둘째 줄이 멀쩡해도 첫 유효 줄로 판정해야 한다
expect_raw 1 'wip: 나쁜 제목\nfeat(a): 좋은 둘째 줄\n' '첫 유효 줄로 판정 (둘째 줄로 구제되지 않는다)'

# 줄 끝 공백: git 의 cleanup 이 지우는 공백을 길이에 세면 안 된다
expect_raw 0 "${b72} \n"                            '72자 + 끝 공백 1개 — git 은 72자로 저장한다'
expect_raw 0 "${b72}  \n"                           '72자 + 끝 공백 2개'
expect_raw 1 "feat: $(printf 'ㄱ%.0s' $(seq 1 67)) \n" '73자 + 끝 공백 — 공백을 깎아도 73자'

# 주석 처리는 두 경로가 **다르다**. 훅은 cleanup 전 원문(템플릿 주석이 아직 있다)을 보고,
# stdin 은 cleanup 이 끝난 최종 텍스트를 본다. 그 비대칭을 여기서 못박는다.
expect_file 0 '# <type>(<scope>): <subject>\n#\n# 주석\nfeat(a): 제목\n' '훅: .gitmessage 템플릿 주석을 건너뛴다'
expect_file 1 '# 주석만 있고 제목이 없다\n#\n'      '훅: 주석만 → 빈 메시지'
expect_raw  1 '#123 급한 WIP\nfeat(a): 본문\n'      'CI: cleanup 끝난 텍스트의 # 는 진짜 subject → 거부'
expect_raw  1 '# 주석처럼 보이는 제목\n'            'CI: # 로 시작하는 한 줄'

# 같은 입력이면 두 경로가 같은 판정을 내려야 한다 (주석 비대칭에 해당하지 않는 입력)
for raw in 'feat(a): 제목\n' 'wip: 제목\n' 'feat(a,b): 제목\n\n본문\n' 'tune(behavior): 임계 조정\n'; do
    rc_s=0; printf '%b' "$raw" | bash "$CHECK" >/dev/null 2>&1 || rc_s=$?
    tmp=$(mktemp); printf '%b' "$raw" > "$tmp"
    rc_f=0; bash "$CHECK" "$tmp" >/dev/null 2>&1 || rc_f=$?
    rm -f "$tmp"
    if [ "$rc_s" = "$rc_f" ]; then
        pass=$((pass + 1))
    else
        fail=$((fail + 1))
        printf '  어긋남: stdin rc=%s != 파일 rc=%s — %s\n' "$rc_s" "$rc_f" "$raw" >&2
    fi
done

# 사용법 오류는 rc 2 — 형식 위반(rc 1)과 구분돼야 한다.
# 여기만 stdin 이 아니라 인자로 부르므로 expect() 를 쓰지 않는다.
echo "--- 사용법 오류는 rc 2"
expect_rc2() {
    local note=$1; shift
    local rc=0
    bash "$CHECK" "$@" </dev/null >/dev/null 2>&1 || rc=$?
    if [ "$rc" = 2 ]; then
        pass=$((pass + 1))
    else
        fail=$((fail + 1))
        printf '  어긋남: rc=%s (기대 2) — %s\n' "$rc" "$note" >&2
    fi
}
expect_rc2 '--help' --help
expect_rc2 '-h' -h
expect_rc2 '없는 파일' /nonexistent/path/xyz
expect_rc2 '디렉토리' "$HERE"
expect_rc2 '인자 2개' /dev/null /dev/null

echo
echo "통과 ${pass} / 어긋남 ${fail}"
[ "$fail" -eq 0 ] || exit 1
