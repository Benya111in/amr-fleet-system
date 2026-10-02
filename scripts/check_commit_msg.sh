#!/usr/bin/env bash
# 커밋 메시지 첫 줄을 Conventional Commits 형식으로 검사한다 (명세 4.1 형상 관리).
#
# 사용법:
#   scripts/check_commit_msg.sh <메시지 파일>                     # git commit-msg 훅 (setup_gitflow.sh 가 등록)
#   git show -s --format=%B <sha> | scripts/check_commit_msg.sh   # CI: stdin 으로 원문 전달 (훅과 같은 첫 줄)
#
# 규칙 (.gitmessage 와 동일):
#   <type>(<scope>)!: <subject>
#   - type   : feat fix docs style refactor perf tune test build ci chore revert
#   - scope  : 선택. 소문자/숫자/'-' '_' '.' '/' 만. 쉼표로 여러 개 (예: itest,scripts)
#   - !      : 선택. breaking change 표시
#   - subject: 72자 이하(초과 시 거부), 50자 이하 권장(초과 시 경고만)
#   - git/GitHub 가 자동 생성하는 Merge / Revert "..." / fixup! / squash! 메시지는 통과
# 종료 코드: 0 통과, 1 위반, 2 사용법 오류
#
# `tune` 과 쉼표 scope 는 Conventional Commits 표준을 벗어나지 않는다 — v1.0.0 은 `feat`/`fix`
# 두 type 의 의미만 규정하고 "그 밖의 type 을 써도 된다"고 명시한다. 명세 4.1 이 요구하는 것은
# 이 형식을 쓰는 것이고, 허용 type 집합은 저장소가 정한다. 아래가 그 결정 근거다.
#   - tune : 파라미터·문턱값 조정. 동작이 바뀌지만 버그 수정(fix)도, 성능 목적(perf)도,
#            동작 불변 구조 개선(refactor)도 아니다. 이 저장소는 측정으로 문턱을 고치는 커밋이
#            많아 따로 둔다 (예: tune(behavior): 마커 모호성 임계 0.05 -> 0.02 rad²).
#   - 쉼표 scope : 한 변경이 여러 패키지에 걸칠 때 (예: feat(itest,scripts)). 항목마다
#            scope 규칙을 그대로 적용한다 — 빈 항목('a,,b' 'a,')은 아래에서 거부한다.
# 규칙을 바꿨으므로 scripts/test_check_commit_msg.sh 가 수용/거부 표를 고정한다
# (CI commit-lint job 이 매 PR 에 돌린다). 규칙을 또 손대면 그 표를 먼저 고친다.
set -u
# 바이트 단위로 다룬다: 글자 수는 아래 count_chars 가 UTF-8 기준으로 따로 센다
export LC_ALL=C

TYPES='feat|fix|docs|style|refactor|perf|tune|test|build|ci|chore|revert'
SUBJECT_MAX=72   # 하드 리밋: GitHub PR/커밋 목록 UI 가 첫 줄을 자르는 경계
SUBJECT_WARN=50  # 권장: git 관례(50/72 규칙)

usage() {
    echo "사용법: $0 [메시지 파일]   (파일이 없으면 stdin 에서 읽는다)" >&2
    exit 2
}

case "${1:-}" in
    -h|--help) usage ;;
esac
[ $# -gt 1 ] && usage

# 메시지 읽기: 파일 인자(훅) 또는 stdin(CI)
# 두 경로는 보는 텍스트가 다르다 — 그 차이가 아래 주석 처리에 쓰인다.
#   훅   : setup_gitflow.sh 가 심은 래퍼가 `bash check_commit_msg.sh "$1"` 로 부른다.
#          git 의 cleanup **전** 원문이라 .gitmessage 템플릿 주석과 줄 끝 공백이 아직 남아 있다.
#   stdin: CI 의 `git show -s --format=%B <sha>` 와 PR 제목. cleanup 이 **끝난** 최종 텍스트다.
if [ $# -eq 1 ]; then
    # 디렉토리나 읽을 수 없는 경로는 빈 메시지(rc 1)가 아니라 사용법 오류(rc 2)다
    if ! { [ -f "$1" ] && [ -r "$1" ]; }; then
        echo "[commit-msg] 파일을 읽을 수 없다: $1" >&2
        exit 2
    fi
    msg=$(cat -- "$1") || { echo "[commit-msg] 파일을 읽을 수 없다: $1" >&2; exit 2; }
    hook_mode=1
else
    msg=$(cat)
    hook_mode=0
fi

# 첫 번째 유효 줄 = 빈 줄(과 훅 경로에서는 주석)을 제외한 첫 줄
TAB=$'\t'
subject=""
skipped_comment=0
while IFS= read -r line || [ -n "$line" ]; do
    line=${line%$'\r'}
    # git 의 cleanup 이 하는 줄 끝 공백 제거를 똑같이 한다. 이걸 빼면 훅이 "72자 + 공백 1개" 를
    # 73자로 세어 거부하는데, git 은 그 공백을 지우고 72자로 저장한다 — 즉 git 이면 합법인 줄을
    # 훅만 막는다 (CI 경로는 이미 cleanup 된 텍스트를 보므로 통과한다 → 두 경로가 엇갈린다).
    # 공백만인 줄은 이 과정에서 빈 줄이 되어 아래 skip 에 자연히 들어간다.
    while :; do
        case "$line" in
            *' ')    line=${line% } ;;
            *"$TAB") line=${line%"$TAB"} ;;
            *)       break ;;
        esac
    done
    case "$line" in
        '') continue ;;
    esac
    # 템플릿 주석 건너뛰기는 **훅 경로에서만** 한다. stdin 으로 온 텍스트는 cleanup 이 끝난
    # 최종 메시지이므로 '#' 로 시작하는 첫 줄은 주석이 아니라 진짜 subject 다 — 그것을 건너뛰면
    # '#123 급한 WIP' 같은 첫 줄이 둘째 줄 덕에 통과해 규칙을 빠져나간다.
    #
    # 훅 경로에도 남는 한계가 있다: 어떤 cleanup 이 걸릴지 훅은 알 수 없다. 에디터 경로는
    # cleanup=strip 이라 git 이 '#' 줄을 지우므로 이 skip 이 git 과 일치하지만, `-m`/`-F` 는
    # cleanup=whitespace 라 git 이 '#' 줄을 **그대로 저장**한다. 그 경우 훅은 통과시키고 CI 가
    # 거부한다 — 방향은 안전하지만(훅이 느슨, CI 가 최종 판정) 조용하면 함정이라 경고를 낸다.
    if [ "$hook_mode" = 1 ]; then
        case "$line" in
            '#'*) skipped_comment=1; continue ;;
        esac
    fi
    subject=$line
    break
done <<EOF
$msg
EOF

hint() {
    cat >&2 <<EOF
[commit-msg] 커밋 메시지 형식 위반: $1
  첫 줄: '$subject'

  형식: <type>(<scope>)!: <subject>
    type    : ${TYPES//|/ }
    scope   : 선택 (소문자/숫자/-_./), 예: navigation, docker
              쉼표로 여러 개도 된다 (예: itest,scripts) — 빈 항목은 안 된다
    !       : 선택, breaking change 표시
    subject : 명령형, 마침표 없이, ${SUBJECT_MAX}자 이하 (${SUBJECT_WARN}자 이하 권장)
  예:
    feat(navigation): A* 글로벌 플래너 직접 구현
    fix(localization): EKF 공분산 초기화 오류 수정
  템플릿: .gitmessage
EOF
    exit 1
}

# UTF-8 문자 수: 연속 바이트(0x80-0xBF)를 제거하면 남는 바이트 수 = 문자 수 (로케일 무관)
count_chars() {
    local t=${1//[$'\x80'-$'\xbf']/}
    printf '%s' "${#t}"
}

[ -n "$subject" ] || hint "메시지가 비어 있다"

# git / GitHub 가 자동 생성하는 메시지는 그대로 통과
case "$subject" in
    'Merge '*)              exit 0 ;;   # Merge branch / Merge pull request #N / Merge remote-tracking branch
    'Revert "'*)            exit 0 ;;   # git revert, GitHub 의 Revert 버튼
    'fixup! '*|'squash! '*) exit 0 ;;   # git commit --fixup/--squash (rebase --autosquash 로 사라진다)
esac

# 헤더 파싱
# 괄호 안에 ',' 를 문자 클래스로 허용하고, 쉼표 구분 항목은 아래에서 따로 검사한다.
# (정규식 하나로 '(a[,a]*)' 를 쓰면 그룹이 하나 늘어 BASH_REMATCH 색인이 밀린다 — desc 가 4 를
#  유지하도록 그룹 수를 바꾸지 않는다.)
re="^(${TYPES})(\([a-z0-9][a-z0-9._/,-]*\))?(!)?: (.*)$"
if ! [[ $subject =~ $re ]]; then
    case "$subject" in
        *': '*) hint "type 이 목록에 없거나 scope 형식이 잘못됐다" ;;
        *':'*)  hint "':' 뒤에 공백이 하나 있어야 한다" ;;
        *)      hint "'<type>: ' 접두어가 없다" ;;
    esac
fi
scope=${BASH_REMATCH[2]}   # '(a)' / '(a,b)' / ''
desc=${BASH_REMATCH[4]}

# 쉼표 scope 의 각 항목은 그 자체로 유효한 scope 여야 한다.
# 위 문자 클래스는 '(a,,b)' '(a,)' '(a,-b)' 를 다 통과시키므로 여기서 막는다.
if [ -n "$scope" ]; then
    inner=${scope#\(}
    inner=${inner%\)}
    case ",${inner}," in
        *,,*) hint "scope 의 쉼표 구분 항목이 비어 있다: '${inner}'" ;;
    esac
    rest=$inner
    while [ -n "$rest" ]; do
        part=${rest%%,*}
        case "$part" in
            [a-z0-9]*) ;;
            *) hint "scope 항목이 소문자/숫자로 시작해야 한다: '${part}'" ;;
        esac
        case "$rest" in
            *,*) rest=${rest#*,} ;;
            *)   rest='' ;;
        esac
    done
fi

[ -n "$desc" ] || hint "subject 가 비어 있다"
case "$desc" in
    ' '*) hint "':' 뒤 공백은 하나만 허용한다" ;;
esac

len=$(count_chars "$subject")
if [ "$len" -gt "$SUBJECT_MAX" ]; then
    hint "첫 줄이 ${len}자 — ${SUBJECT_MAX}자를 넘는다"
fi
if [ "$len" -gt "$SUBJECT_WARN" ]; then
    echo "[commit-msg] 경고: 첫 줄이 ${len}자 — ${SUBJECT_WARN}자 이하를 권장한다" >&2
fi
case "$desc" in
    *.) echo "[commit-msg] 경고: subject 끝의 마침표는 관례상 생략한다" >&2 ;;
esac

# '#' 줄을 건너뛰고 판정했다면, git 이 그 줄을 정말 지울지는 cleanup 모드에 달렸다.
# 에디터 경로(cleanup=strip)는 지우지만 `-m`/`-F`(cleanup=whitespace)는 그대로 저장한다 —
# 그러면 저장된 첫 줄이 '#...' 이 되어 CI commit-lint 가 거부한다. 조용히 넘기지 않는다.
if [ "$skipped_comment" = 1 ]; then
    cat >&2 <<EOF
[commit-msg] 경고: '#' 로 시작하는 줄을 건너뛰고 '$subject' 로 판정했다.
  에디터로 커밋했다면(cleanup=strip) git 이 그 줄을 지우므로 문제가 없다.
  그런데 -m / -F 로 커밋했다면 git 은 '#' 줄을 그대로 저장하고, 그러면 CI commit-lint 가
  그 첫 줄을 거부한다. 그런 경우라면 --cleanup=strip 을 주거나 '#' 줄을 지워라.
EOF
fi

exit 0
