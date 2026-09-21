#!/usr/bin/env bash
# 커밋 메시지 첫 줄을 Conventional Commits 형식으로 검사한다 (명세 4.1 형상 관리).
#
# 사용법:
#   scripts/check_commit_msg.sh <메시지 파일>              # git commit-msg 훅 (setup_gitflow.sh 가 등록)
#   git log -1 --format=%B | scripts/check_commit_msg.sh   # CI: stdin 으로 메시지 전달
#
# 규칙 (.gitmessage 와 동일):
#   <type>(<scope>)!: <subject>
#   - type   : feat fix docs style refactor perf test build ci chore revert
#   - scope  : 선택. 소문자/숫자/'-' '_' '.' '/' 만
#   - !      : 선택. breaking change 표시
#   - subject: 72자 이하(초과 시 거부), 50자 이하 권장(초과 시 경고만)
#   - git/GitHub 가 자동 생성하는 Merge / Revert "..." / fixup! / squash! 메시지는 통과
# 종료 코드: 0 통과, 1 위반, 2 사용법 오류
set -u
# 바이트 단위로 다룬다: 글자 수는 아래 count_chars 가 UTF-8 기준으로 따로 센다
export LC_ALL=C

TYPES='feat|fix|docs|style|refactor|perf|test|build|ci|chore|revert'
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

# 메시지 읽기: 파일 인자 또는 stdin
if [ $# -eq 1 ]; then
    [ -r "$1" ] || { echo "[commit-msg] 파일을 읽을 수 없다: $1" >&2; exit 2; }
    msg=$(cat -- "$1")
else
    msg=$(cat)
fi

# 첫 번째 유효 줄 = 주석(#)과 빈 줄을 제외한 첫 줄
# (commit.template 의 주석은 commit-msg 훅 실행 시점에 아직 남아 있다)
subject=""
while IFS= read -r line || [ -n "$line" ]; do
    line=${line%$'\r'}
    case "$line" in
        ''|'#'*) continue ;;
    esac
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
re="^(${TYPES})(\([a-z0-9][a-z0-9._/-]*\))?(!)?: (.*)$"
if ! [[ $subject =~ $re ]]; then
    case "$subject" in
        *': '*) hint "type 이 목록에 없거나 scope 형식이 잘못됐다" ;;
        *':'*)  hint "':' 뒤에 공백이 하나 있어야 한다" ;;
        *)      hint "'<type>: ' 접두어가 없다" ;;
    esac
fi
desc=${BASH_REMATCH[4]}

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

exit 0
