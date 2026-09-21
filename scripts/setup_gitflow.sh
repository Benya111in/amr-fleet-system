#!/usr/bin/env bash
# Git Flow 로컬 환경 초기화 (명세 4.1: Main / Develop / Feature + conventional commits)
#
# develop / main 은 GitHub 룰셋으로 보호된다 (PR 필수, develop 승인 1명 · main 승인 2명, force-push 금지).
# 따라서 로컬에서 develop 에 직접 병합하지 않고, 이 스크립트는 다음만 한다:
#   1. origin 을 fetch 하고 로컬 develop 이 origin/develop 을 추적하도록 맞춘다
#   2. .gitmessage 를 commit.template 으로 등록한다
#   3. scripts/check_commit_msg.sh 를 commit-msg 훅으로 설치한다
# 어느 디렉토리에서 실행해도 되고, 여러 번 실행해도 결과는 같다.
set -euo pipefail

cd "$(dirname "$0")/.."
ROOT=$(git rev-parse --show-toplevel)
cd "$ROOT"

log()  { echo "[gitflow] $*"; }
warn() { echo "[gitflow] 경고: $*" >&2; }

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
    # fast-forward 만 시도한다. 갈라졌으면 손대지 않고 알려만 준다.
    if [ "$current" = "develop" ]; then
        if git merge -q --ff-only origin/develop 2>/dev/null; then
            log "develop = origin/develop ($(git rev-parse --short develop))"
        else
            warn "develop 이 origin/develop 과 갈라졌다. develop 에는 직접 커밋하지 말고 PR 로 올려라."
        fi
    else
        if git fetch -q origin develop:develop 2>/dev/null; then
            log "develop = origin/develop ($(git rev-parse --short develop))"
        else
            warn "로컬 develop 을 fast-forward 하지 못했다 (다른 워크트리에서 체크아웃 중이거나 갈라짐)."
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
# 훅 본체는 저장소의 scripts/check_commit_msg.sh 이고, .git/hooks 에는 그것을 부르는 래퍼만 둔다.
# (경로에 의존하지 않아 저장소를 옮기거나 워크트리에서 커밋해도 동작한다)
hooks_dir=$(git rev-parse --git-path hooks)
mkdir -p "$hooks_dir"
hook="$hooks_dir/commit-msg"
marker="# amr-fleet-system commit-msg hook (scripts/setup_gitflow.sh 가 생성)"

if [ -e "$hook" ] && ! grep -qF "$marker" "$hook" 2>/dev/null; then
    backup="$hook.bak.$(date +%Y%m%d%H%M%S)"
    mv "$hook" "$backup"
    warn "기존 commit-msg 훅을 백업했다: $backup"
fi

cat > "$hook" <<EOF
#!/usr/bin/env bash
$marker
exec "\$(git rev-parse --show-toplevel)/scripts/check_commit_msg.sh" "\$1"
EOF
chmod +x "$hook" scripts/check_commit_msg.sh
log "commit-msg 훅 설치: $hook -> scripts/check_commit_msg.sh"

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
  → 리뷰 승인 1명 + CI 통과 후 병합

릴리스:
  gh pr create --base main --head develop
  → 승인 2명 + CI 통과 후 병합

커밋 메시지:
  <type>(<scope>): <subject>   (.gitmessage 참고, commit-msg 훅이 검사)
EOF
