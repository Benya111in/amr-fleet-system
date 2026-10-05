#!/usr/bin/env python3
"""
README.md 의 "## 6. 결과" 절을 TLC 로그에서 다시 만든다.

사용법: python3 formal/fleet/update_readme_results.py [로그 디렉터리]
표 값은 summarize.py 가 로그 원문에서 뽑는다 (손으로 옮겨 적지 않는다).
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = sys.argv[1] if len(sys.argv) > 1 else '/home/CAPYI_AI/logs/agents/f2-tla/tlc'

GROUPS = [
    ('6.1 RA_LiveMembers (생존 멤버 RA 변형)',
     ['RA_report_N5', 'RA_dropOne_N5', 'RA_dropOneSided_N5', 'RA_any_N5', 'RA_fixed_N5',
      'RA_fixed_N5_r3', 'RA_fixed_N3']),
    ('6.2 QuorumVote 설계 (Mutation none, Variant report) — 안전',
     ['QV_N3_Z1_R2_C1', 'QV_N3_Z1_R2_C1_view', 'QV_N3_Z1_R2_C2_view', 'QV_N3_Z2_R3_C1_view',
      'QV_N5_Z1_R2_C1_view', 'QV_N5_Z2_R2_C1_view']),
    ('6.3 QuorumVote 가드 변이',
     ['QV_N3_Z1_R2_C1_forget', 'QV_N5_Z1_R2_C1_forget_view', 'QV_N3_Z1_R2_C0_expireCommitted',
      'QV_N3_Z1_R2_C0_smallQuorum', 'QV_N3_Z1_R2_C0_onePhase', 'QV_N3_Z1_R3_C0_noRidCheck',
      'QV_N3_Z1_R3_C1_volatileInc', 'QV_N3_Z2_R2_C0_overwriteTentative',
      'QV_N3_Z2_R3_C1_overwriteTentative_view']),
    ('6.4 QuorumVote 활성 결함 (ABANDON 처리)',
     ['QV_N3_Z1_R2_C1_strand_report_view', 'QV_N3_Z1_R2_C1_strand_fixA_view',
      'QV_N3_Z1_R2_C1_strand_fixB_view', 'QV_N3_Z1_R2_C1_dead_report_view',
      'QV_N3_Z1_R2_C1_dead_fixA_view', 'QV_N3_Z1_R2_C1_dead_fixB_view',
      'QV_N3_Z2_R2_C1_fixB_view', 'QV_N3_Z1_R3_C1_fixB_view']),
    ('6.5 TaskLock',
     ['TL_N3_T1_R2_C1', 'TL_N3_T1_R2_C1_view', 'TL_N5_T1_R2_C1_view', 'TL_N3_T2_R3_C1_view',
      'TL_N3_T1_R2_C1_forget', 'TL_N3_T1_R2_C0_expireCommitted',
      'TL_N3_T1_R2_C0_releaseAfterDone']),
    ('6.6 중단한 실행 (결과로 쓰지 않는다)',
     ['QV_N3_Z2_R3_C1.noview-killed', 'QV_N3_Z2_R3_C0_noRidCheck.outer-timeout-killed',
      'QV_N3_Z2_R3_C1_volatileInc.outer-timeout-killed', 'TL_N3_T2_R3_C1.noview-killed',
      'TL_N5_T1_R2_C1.noview-killed', 'QV_N3_Z2_R3_C1_fixB_view.reprioritized-killed']),
]

HEAD = """## 6. 결과

[측정] 모든 수치는 `summarize.py` 가 로그(`/home/CAPYI_AI/logs/agents/f2-tla/tlc/<cfg>.log`)에서
뽑은 것이다 (이 절은 `update_readme_results.py` 가 다시 만든다).
cfg 이름: `<모델>_N<로봇>_Z<구역>|T<작업>_R<전체 요청 상한>_C<전체 크래시 상한>[_<변이/변형>][_view]`.
"wall s" 는 공유 서버(32 코어, 시작 1 분 loadavg 열 참고) 위 4 작업자 기준 **예비값**이다.
"깊이" 는 위반이면 반례까지의 BFS 깊이, 위반 없음이면 상태 그래프 전체의 깊이다.
spec sha256 열의 스펙 판은 커밋으로 찾을 수 있다 (§6.7).

"""

MEMO = """### 6.7 스펙 판과 실행 메모

| spec sha256[:12] | 파일 @ 커밋 | 앞 판과의 차이 |
|---|---|---|
| e88070dac980 | RA_LiveMembers.tla @ ba96132 | — (표에 없음: 아래 판으로 다시 돌림) |
| a4a6bca35ac2 | RA_LiveMembers.tla @ 1b67bc7 | `ViewChangers` 상수 추가 |
| 1a25ba07b50a | QuorumVote.tla @ b9a534b | — |
| 886be4a88678 | QuorumVote.tla @ 68fcc1d | `LiveMsg`·`View` 정의 추가만 (Init/Next 동일) |
| 1ab2620fb436 | QuorumVote.tla @ e65d6b5 | `NoDeadZone` 정의 추가만 (Init/Next 동일) |
| 290b7fafd0fa | TaskLock.tla @ 68fcc1d | — |
| 7ec69a1f8222 | TaskLock.tla @ ab7cdaf | `LiveMsg`·`View` 정의 추가만 (Init/Next 동일) |

QuorumVote·TaskLock 판 사이의 차이는 `git diff` 로 확인했고 지운 줄이 없다 (정의 추가만).
그래서 같은 cfg 의 상태 수는 판과 무관하다. RA 는 `ChangeView` 가드가 `Requesters` 에서
`ViewChangers` 로 바뀌었고, `ViewChangers = Requesters` 인 cfg 는 앞 판과 Next 가 같다
(위반 없음 실행 셋의 상태 수가 두 판에서 같았다).

- `QV_N3_Z2_R3_C1_view` 로그 끝의 `rc=127` 은 TLC 가 아니라 **실행 스크립트** 쪽이다.
  실행 중에 `run_tlc.sh` 를 고쳐서 bash 가 바뀐 파일의 엉뚱한 줄
  (`h: command not found`)을 읽었다. TLC 자신은 "Model checking completed. No error
  has been found." 와 최종 통계를 찍고 끝났다 (29 분 26 초). 이 실행은 입력 사본
  (`inputs/`) 기능이 들어가기 전이라 사본이 없고, 스펙은 sha 로 68fcc1d 의 파일과 같다.
- 같은 실행의 지문 충돌 확률 추정: optimistic 5.4E-4, actual fingerprints 1.6E-5
  (TLC 출력). 다른 실행의 값은 각 로그에 있다.
- VIEW 없이 돌린 `QV_N3_Z1_R2_C1`(136,276) 과 VIEW 판(30,223)은 둘 다 위반 없음이다.
  깊이가 32 → 29 로 줄어드는 것은 VIEW 가 같다고 본 상태를 합치기 때문이다.
- `*.reprioritized-killed` 는 더 중요한 실행에 CPU 를 넘기려고 직접 멈춘 것이다.
"""


def main():
    out = [HEAD]
    for title, names in GROUPS:
        table = subprocess.run(
            [sys.executable, os.path.join(HERE, 'summarize.py'), LOGS] + names,
            capture_output=True, text=True, check=True).stdout
        out.append('### %s\n\n%s\n' % (title, table))
    out.append(MEMO)
    path = os.path.join(HERE, 'README.md')
    with open(path, encoding='utf-8') as fh:
        text = fh.read()
    start = text.index('## 6. 결과')
    end = text.index('## 7. ')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text[:start] + ''.join(out) + '\n' + text[end:])


if __name__ == '__main__':
    main()
