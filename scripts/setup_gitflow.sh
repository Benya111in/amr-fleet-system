#!/usr/bin/env bash
# Git Flow 브랜치 전략 초기화 (명세 4장: Main, Develop, Feature)
set -euo pipefail

cd "$(dirname "$0")/.."

# 최초 커밋이 없으면 develop 을 만들 수 없다.
if ! git rev-parse HEAD >/dev/null 2>&1; then
    echo "[gitflow] 커밋이 없다. 먼저 초기 커밋을 만들어라."
    exit 1
fi

git rev-parse --verify develop >/dev/null 2>&1 \
    || git branch develop
echo "[gitflow] main / develop 준비 완료"

# conventional commits 템플릿 등록
git config commit.template .gitmessage
echo "[gitflow] 커밋 메시지 템플릿 등록 (.gitmessage)"

cat <<'EOF'

브랜치 전략
  main      릴리스만 병합. 직접 커밋 금지.
  develop   통합 브랜치. feature 가 여기로 병합된다.
  feature/* 기능 단위 작업 브랜치.

새 기능 시작:
  git checkout develop && git checkout -b feature/<이름>

작업 완료 후:
  git checkout develop && git merge --no-ff feature/<이름>
EOF
