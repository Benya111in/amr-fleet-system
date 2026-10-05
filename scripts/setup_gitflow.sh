#!/usr/bin/env bash
# Git Flow 로컬 환경 초기화 (명세 4.1: Main / Develop / Feature + conventional commits)
#
# develop / main 은 GitHub 룰셋으로 보호된다 (PR 필수, develop 승인 1명 · main 승인 2명, force-push 금지).
# 따라서 로컬에서 develop 에 직접 병합하지 않고, 이 스크립트는 다음만 한다:
#   1. origin 을 fetch 하고 로컬 develop 이 origin/develop 을 추적하도록 맞춘다
#   2. .gitmessage 를 commit.template 으로 등록한다
#   3. scripts/check_commit_msg.sh 를 부르는 commit-msg 훅을 이 저장소의 훅 디렉토리에 설치한다
#      (저장소 밖을 가리키는 core.hooksPath — 전역 훅 등 — 에는 설치하지 않고 안내만 하고 끝낸다)
# 어느 디렉토리에서 실행해도 되고, 여러 번 실행해도 결과는 같다.
set -euo pipefail

cd "$(dirname "$0")/.."
ROOT=$(git rev-parse --show-toplevel)
cd "$ROOT"

log()  { echo "[gitflow] $*"; }
warn() { echo "[gitflow] 경고: $*" >&2; }
first_line() { printf '%s\n' "$1" | head -n1; }

# 최초 커밋이 없으면 브랜치를 만들 수 없다.
if ! git rev-parse --verify -q HEAD >/dev/null; then
    echo "[gitflow] 커밋이 없다. 먼저 초기 커밋을 만들어라." >&2
    exit 1
fi

# ---------------------------------------------------------------- 1. develop
current=$(git symbolic-ref -q --short HEAD || echo "(detached)")

if git remote get-url origin >/dev/null 2>&1; then
    log "origin fetch: $(git remote get-url origin)"
    git fetch --prune origin || warn "fetch 실패 — 네트워크/권한을 확인하라. 로컬 정보로 계속한다."
else
    warn "origin 원격이 없다. 원격을 추가한 뒤 다시 실행하라:"
    warn "  git remote add origin git@github.com:Benya111in/amr-fleet-system.git"
fi

if git rev-parse --verify -q refs/remotes/origin/develop >/dev/null; then
    if ! git rev-parse --verify -q refs/heads/develop >/dev/null; then
        git branch --track develop origin/develop
        log "로컬 develop 생성 (origin/develop 추적)"
    fi
    upstream=$(git rev-parse --abbrev-ref -q 'develop@{upstream}' 2>/dev/null || true)
    if [ "$upstream" != "origin/develop" ]; then
        git branch --set-upstream-to=origin/develop develop
        log "develop 의 upstream 을 origin/develop 으로 설정"
    fi
    # fast-forward 만 시도한다. 갈라짐(로컬에만 있는 커밋)은 먼저 따로 판정해 손대지 않고 알려만 주고,
    # 그 밖의 실패(수정 중인 파일과 충돌, 다른 워크트리에서 체크아웃 중)는 git 의 실제 이유를 보여준다.
    if ! git merge-base --is-ancestor develop origin/develop; then
        warn "develop 이 origin/develop 과 갈라졌다 (로컬에만 있는 커밋 $(git rev-list --count origin/develop..develop)개)." \
             "develop 에는 직접 커밋하지 말고 PR 로 올려라."
    elif [ "$current" = "develop" ]; then
        if err=$(git merge -q --ff-only origin/develop 2>&1); then
            log "develop = origin/develop ($(git rev-parse --short develop))"
        else
            warn "develop 을 fast-forward 하지 못했다: $(first_line "$err")"
        fi
    else
        if err=$(git fetch -q origin develop:develop 2>&1); then
            log "develop = origin/develop ($(git rev-parse --short develop))"
        else
            warn "로컬 develop 을 fast-forward 하지 못했다: $(first_line "$err")"
        fi
    fi
else
    # 원격에 develop 이 아직 없는 초기 상태: 로컬에서만 만들고 push 는 사람이 한다.
    if ! git rev-parse --verify -q refs/heads/develop >/dev/null; then
        git branch develop
        log "origin/develop 이 없어 현재 HEAD 에서 로컬 develop 생성"
    fi
    warn "원격에 develop 이 없다. 처음 한 번은 직접 올려라:  git push -u origin develop"
fi

# ---------------------------------------------------------------- 2. 커밋 템플릿
# 상대 경로: git 은 명령 실행 전에 워크트리 최상위로 이동하므로 하위 디렉토리/다른 워크트리에서도 풀린다
git config commit.template .gitmessage
log "커밋 메시지 템플릿 등록 (.gitmessage)"

# ---------------------------------------------------------------- 3. commit-msg 훅
# 훅 본체는 저장소의 scripts/check_commit_msg.sh 이고, 훅 디렉토리에는 그것을 부르는 래퍼만 둔다.
# (경로에 의존하지 않아 저장소를 옮기거나 워크트리에서 커밋해도 동작한다)
#
# 설치 위치는 이 저장소의 공용 훅 디렉토리(.git/hooks — 연결된 워크트리도 공유)이거나, 저장소 안을
# 가리키는 core.hooksPath 다. 저장소 밖(예: git config --global core.hooksPath ~/.githooks)에 설치하면
# 이 저장소 전용 래퍼가 다른 모든 저장소의 커밋까지 검사하게 되므로 거부한다.
common_dir=$(cd "$(git rev-parse --git-common-dir)" && pwd -P)
hooks_path=$(git config --type=path --get core.hooksPath || true)
if [ -n "$hooks_path" ]; then
    # 상대 경로는 워크트리 최상위 기준 (git 은 훅을 그 디렉토리에서 실행한다)
    case "$hooks_path" in
        /*) hooks_dir=$hooks_path ;;
        *)  hooks_dir="$ROOT/$hooks_path" ;;
    esac
    hooks_dir=$(realpath -m -- "$hooks_dir")
    case "$hooks_dir/" in
        "$ROOT"/*|"$common_dir"/*)
            log "core.hooksPath 사용: $hooks_path" ;;
        *)
            scope=$(git config --show-scope --get core.hooksPath 2>/dev/null | cut -f1 || true)
            case "${scope:-local}" in
                global)   unset_cmd="git config --global --unset core.hooksPath" ;;
                system)   unset_cmd="sudo git config --system --unset core.hooksPath" ;;
                worktree) unset_cmd="git config --worktree --unset core.hooksPath" ;;
                *)        unset_cmd="git config --unset core.hooksPath" ;;
            esac
            cat >&2 <<EOF
[gitflow] 오류: core.hooksPath 가 저장소 밖을 가리킨다 (${scope:-local} 설정): $hooks_path
  거기에 설치하면 이 저장소 전용 commit-msg 훅이 다른 모든 저장소의 커밋까지 막게 되므로 설치하지 않는다.
  다음 중 하나로 정리한 뒤 다시 실행하라:
    git config core.hooksPath "$common_dir/hooks"   # 이 저장소만 기본 훅 디렉토리를 쓰도록 로컬 설정으로 덮어쓴다
    $unset_cmd   # ${scope:-local} 설정 해제 → 기본 .git/hooks 사용
  지금 쓰는 전역 훅이 이 저장소에도 필요하면 그 commit-msg 파일을 위 디렉토리에 복사해 둬라
  (이 스크립트가 commit-msg.local 로 옮겨 형식 검사 뒤에 이어서 실행한다).
EOF
            exit 1 ;;
    esac
else
    hooks_dir="$common_dir/hooks"
fi
mkdir -p "$hooks_dir"
hook="$hooks_dir/commit-msg"
local_hook="$hooks_dir/commit-msg.local"
marker="# amr-fleet-system commit-msg hook (scripts/setup_gitflow.sh 가 생성)"

# 이미 있는 남의 훅: 실행 가능하면 commit-msg.local 로 옮겨 형식 검사 뒤에 이어서 실행되게 하고,
# 실행 권한이 없어 git 이 무시하던 파일이면 백업만 남긴다.
if { [ -e "$hook" ] || [ -L "$hook" ]; } && ! grep -qF "$marker" "$hook" 2>/dev/null; then
    if [ -x "$hook" ]; then
        if [ -e "$local_hook" ] || [ -L "$local_hook" ]; then
            backup="$local_hook.bak.$(date +%Y%m%d%H%M%S)"
            mv "$local_hook" "$backup"
            warn "이전 commit-msg.local 을 백업했다: $backup"
        fi
        mv "$hook" "$local_hook"
        warn "기존 commit-msg 훅을 $local_hook 로 옮겼다 — 형식 검사 뒤에 이어서 실행된다"
    else
        backup="$hook.bak.$(date +%Y%m%d%H%M%S)"
        mv "$hook" "$backup"
        warn "실행 권한이 없어 git 이 무시하던 commit-msg 훅을 백업했다: $backup"
    fi
fi

# 래퍼: 검사 스크립트가 없는 체크아웃(main, 옛 브랜치, bisect)은 막지 않는다.
# bash 로 실행하므로 스크립트의 실행 권한에 의존하지 않는다.
cat > "$hook" <<EOF
#!/usr/bin/env bash
$marker
f="\$(git rev-parse --show-toplevel)/scripts/check_commit_msg.sh"
if [ -f "\$f" ]; then
    bash "\$f" "\$1" || exit \$?
fi
# setup_gitflow.sh 가 옮겨 둔 기존 훅이 있으면 이어서 실행한다
local_hook="\$(dirname -- "\$0")/commit-msg.local"
[ -x "\$local_hook" ] && exec "\$local_hook" "\$1"
exit 0
EOF
chmod +x "$hook"
log "commit-msg 훅 설치: $hook -> scripts/check_commit_msg.sh"
if [ -x "$local_hook" ]; then
    log "  형식 검사 뒤에 이어서 실행: $local_hook"
fi

# ---------------------------------------------------------------- 안내
cat <<'EOF'

브랜치 전략 (GitHub 룰셋으로 강제)
  main      릴리스만. develop → main PR, 승인 2명. 직접 push 금지.
  develop   통합 브랜치. feature → develop PR, 승인 1명. 직접 push 금지.
  feature/* 기능 단위 작업 브랜치. develop 에서 분기.

새 기능 시작:
  git switch develop && git pull --ff-only
  git switch -c feature/<이름>

작업 완료 후:
  git push -u origin feature/<이름>
  gh pr create --base develop            # 또는 GitHub 웹에서 PR (템플릿 자동 적용)
  → 리뷰 승인 1명 + CI(commit-lint, build-test) 통과 후 병합
    (PR 제목도 커밋 형식 — "Squash and merge" 시 커밋 첫 줄이 된다)

릴리스:
  gh pr create --base main --head develop
  → 승인 2명 + CI 통과 후 병합

커밋 메시지:
  <type>(<scope>): <subject>   (.gitmessage 참고, commit-msg 훅과 CI 가 검사)
EOF
