#!/usr/bin/env python3
"""
보너스 1 실험 노트·실행 목록 도구.

나중에 다시 조사할 때 "무엇을, 어떤 코드·설정·시드로, 어떤 조건에서 돌려, 무엇이 나왔고, 어디에 원자료가 있나"를
한 곳에서 찾게 하려고 만든다. 순수 Python 표준 라이브러리만 쓴다.

하위 명령:
  new-entry   실험 노트 항목 파일을 틀에서 만든다 (docs/research/rl-navigation/notebook/entries/)
  register    실행 한 건을 실행 목록(runs/<agent>.jsonl)에 덧붙인다 — 필수 필드를 검사한다
  index       entries/ 와 runs/*.jsonl 로 index.md 를 다시 만든다
  check       실행 목록의 필수 필드·경로 존재·중복 id 를 검사한다
"""
import argparse
import datetime as dt
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
NB = os.path.join(REPO, 'docs', 'research', 'rl-navigation', 'notebook')
ENTRIES = os.path.join(NB, 'entries')
RUNS_DIR = os.path.join(NB, 'runs')  # 에이전트마다 runs/<agent>.jsonl — 동시 추가가 merge 충돌을 내지 않게

# 실행 목록의 필수 필드. 값이 없으면 등록을 거부한다 — 나중에 출처를 잃지 않으려고.
REQUIRED = {
    'run_id': '전역 유일 id (예: B1_ppo_v12_s3)',
    'stage': '단계 (예: A3, B1)',
    'agent': '실행한 에이전트 id',
    'kind': 'train | eval | probe | gazebo | test',
    'sim': '2d | isaac | gazebo | offline',
    'commit': '코드 커밋 (git rev-parse --short)',
    'dirty': '미커밋 변경 여부 (true/false)',
    'config': '설정 요약 또는 config 파일 경로',
    'seed': '시드 (정수 또는 목록)',
    'started': 'ISO 시각',
    'ended': 'ISO 시각',
    'raw_dir': '원자료 디렉터리 (절대 경로)',
    'result': '핵심 결과 dict (판정 지표, n 포함)',
    'label': '[측정] | [계산] | 미측정 | 실패',
    'load': '시작·끝 loadavg 또는 "n/a"',
}


def _git(*args):
    out = subprocess.run(['git', '-C', REPO, *args], capture_output=True, text=True)
    return out.stdout.strip()


def cmd_new_entry(a):
    os.makedirs(ENTRIES, exist_ok=True)
    now = dt.datetime.now()
    slug = re.sub(r'[^0-9A-Za-z가-힣_-]+', '-', a.slug).strip('-')[:60]
    path = os.path.join(ENTRIES, f"{now:%Y-%m-%d_%H%M}_{a.agent}_{slug}.md")
    if os.path.exists(path):
        sys.exit(f'이미 있다: {path}')
    body = f"""# {a.title}

- 시각: {now:%Y-%m-%d %H:%M} KST · 에이전트: {a.agent} · 단계: {a.stage}
- 코드: `{_git('rev-parse', '--short', 'HEAD')}` (브랜치 `{_git('rev-parse', '--abbrev-ref', 'HEAD')}`)
- 관련 실행 id: (runs/<에이전트>.jsonl 의 run_id 목록)

## 질문
(이 실험이 답하려는 것 한 문장)

## 가설과 예측
(무엇을 예상하는가, 무엇이 나오면 가설을 버리는가 — 결과를 보기 전에 적는다)

## 설정
(시뮬레이터, 계약 판, 보상·비용, DR, 시드, 스텝, 평가 규칙(판정 대상·평가 seed·n), 이미지, 명령)

## 결과
(표: 시드별 값, 실패 시드 수. [측정]/[계산]/미측정 표기. 원자료 경로)

## 해석
(결과가 말하는 것과 말하지 않는 것. 공유 부하 하 예비값 여부)

## 다음
(이 결과로 바뀌는 결정, 남은 질문)
"""
    with open(path, 'w', encoding='utf-8') as f:
        f.write(body)
    print(path)


def _load_runs():
    out = []
    if not os.path.isdir(RUNS_DIR):
        return out
    for fn in sorted(os.listdir(RUNS_DIR)):
        if not fn.endswith('.jsonl'):
            continue
        with open(os.path.join(RUNS_DIR, fn), encoding='utf-8') as f:
            for i, line in enumerate(f, 1):
                line = line.strip()
                if line:
                    try:
                        out.append(json.loads(line))
                    except json.JSONDecodeError as e:
                        sys.exit(f'runs/{fn} {i}행 JSON 오류: {e}')
    return out


def _validate(rec):
    missing = [k for k in REQUIRED if k not in rec or rec[k] in (None, '')]
    if missing:
        return f'필수 필드 없음: {missing}'
    if rec['kind'] not in ('train', 'eval', 'probe', 'gazebo', 'test'):
        return f"kind 값 오류: {rec['kind']}"
    if rec['sim'] not in ('2d', 'isaac', 'gazebo', 'offline'):
        return f"sim 값 오류: {rec['sim']}"
    if not isinstance(rec['result'], dict):
        return 'result 는 dict 여야 한다'
    if not os.path.isabs(rec['raw_dir']):
        return 'raw_dir 는 절대 경로여야 한다'
    return None


def cmd_register(a):
    rec = json.loads(a.json) if a.json else json.load(open(a.file, encoding='utf-8'))
    err = _validate(rec)
    if err:
        sys.exit(err)
    if any(r['run_id'] == rec['run_id'] for r in _load_runs()):
        sys.exit(f"run_id 중복: {rec['run_id']}")
    rec.setdefault('registered', dt.datetime.now().isoformat(timespec='seconds'))
    os.makedirs(RUNS_DIR, exist_ok=True)
    agent = re.sub(r'[^0-9A-Za-z_-]+', '-', rec['agent'])
    with open(os.path.join(RUNS_DIR, f'{agent}.jsonl'), 'a', encoding='utf-8') as f:
        f.write(json.dumps(rec, ensure_ascii=False) + '\n')
    print('registered', rec['run_id'])


def cmd_check(a):
    runs = _load_runs()
    seen, bad = set(), 0
    for r in runs:
        err = _validate(r)
        if err:
            print(f"{r.get('run_id')}: {err}")
            bad += 1
        if r.get('run_id') in seen:
            print(f"{r.get('run_id')}: 중복")
            bad += 1
        seen.add(r.get('run_id'))
        if a.paths and not os.path.isdir(r.get('raw_dir', '')):
            print(f"{r.get('run_id')}: raw_dir 없음 {r.get('raw_dir')}")
            bad += 1
    print(f'{len(runs)} 건, 문제 {bad}')
    sys.exit(1 if bad else 0)


def cmd_index(a):
    files = sorted(os.listdir(ENTRIES)) if os.path.isdir(ENTRIES) else []
    runs = _load_runs()
    lines = ['# 보너스 1 실험 노트 색인', '',
             '> `python3 rl/tools/notebook.py index` 가 만든다 — 손으로 고치지 않는다.', '',
             '## 노트 항목 (시간순)', '',
             '| 시각 | 에이전트 | 제목 | 파일 |', '|---|---|---|---|']
    for fn in files:
        if not fn.endswith('.md'):
            continue
        with open(os.path.join(ENTRIES, fn), encoding='utf-8') as f:
            title = f.readline().lstrip('# ').strip()
        parts = fn.split('_', 3)
        when = f'{parts[0]} {parts[1][:2]}:{parts[1][2:]}' if len(parts) > 2 else ''
        agent = parts[2] if len(parts) > 2 else ''
        lines.append(f'| {when} | {agent} | {title} | [entries/{fn}](entries/{fn}) |')
    lines += ['', f'## 실행 목록 요약 ({len(runs)} 건, 원본 `runs/<에이전트>.jsonl`)', '',
              '| run_id | 단계 | 종류 | 시뮬 | 커밋 | 시드 | 핵심 결과 | 표기 |',
              '|---|---|---|---|---|---|---|---|']
    for r in runs:
        res = ', '.join(f'{k}={v}' for k, v in list(r.get('result', {}).items())[:4])
        lines.append(f"| {r['run_id']} | {r['stage']} | {r['kind']} | {r['sim']} | "
                     f"{r['commit']} | {r['seed']} | {res} | {r['label']} |")
    with open(os.path.join(NB, 'index.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print(os.path.join(NB, 'index.md'))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = p.add_subparsers(dest='cmd', required=True)
    e = sp.add_parser('new-entry')
    e.add_argument('--agent', required=True)
    e.add_argument('--stage', required=True)
    e.add_argument('--slug', required=True)
    e.add_argument('--title', required=True)
    e.set_defaults(f=cmd_new_entry)
    r = sp.add_parser('register')
    g = r.add_mutually_exclusive_group(required=True)
    g.add_argument('--json')
    g.add_argument('--file')
    r.set_defaults(f=cmd_register)
    i = sp.add_parser('index')
    i.set_defaults(f=cmd_index)
    c = sp.add_parser('check')
    c.add_argument('--paths', action='store_true')
    c.set_defaults(f=cmd_check)
    a = p.parse_args()
    a.f(a)


if __name__ == '__main__':
    main()
