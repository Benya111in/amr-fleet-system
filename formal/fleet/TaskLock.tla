------------------------------ MODULE TaskLock ------------------------------
(***************************************************************************)
(* 작업 ID 정족수 잠금 -- "실행권" (중복 실행 0).                           *)
(*                                                                         *)
(* 출처: 보고서 「분산 플릿 관리 협상 할당과 폴백」 1행 요약("실제 픽업을   *)
(* 시작할 실행권은 구역과 같은 투표 모듈로 작업 ID 를 잠가 확정한다"),       *)
(* 쓸 것 표의 "실행권" 행("작업 ID 정족수 잠금(잠정 -> 픽업 시작 시 확정,   *)
(* 픽업 후 고장은 FAILED)"), 주장 표의 "손실이 있어도 중복 실행이 0".       *)
(*                                                                         *)
(* QuorumVote 의 투표를 구역 대신 작업 id 에 쓴다. 다른 점:                *)
(*  - 요청은 작업 하나. 사슬 없음.                                         *)
(*  - "진입" 대신 "픽업 시작" (Start). 시작은 되돌릴 수 없다.              *)
(*  - 시작한 작업의 잠금은 풀지 않는다 (완료든 FAILED 든 묘비로 남는다).    *)
(*    시작 전에는 RELEASE 로 내려놓을 수 있다 (협상이 다른 로봇에게 줌).   *)
(*  - 어느 로봇이 어느 작업을 잠그려 하는지는 비결정 -- 협상(CBAA)이       *)
(*    손실·지연으로 두 로봇에게 같은 작업을 "낙찰" 시키는 경우를 다 덮는다. *)
(*                                                                         *)
(* 네트워크·고장 모델은 QuorumVote 와 같다 (지우지 않는 메시지 집합,         *)
(* 크래시-재시작 한 단계, 영속 = 잠금 표·incarnation, 비잔틴 없음).         *)
(* 시작한 뒤 크래시하면 그 작업은 FAILED 로 남고 로봇은 idle 로 돌아온다.  *)
(* 시작 전 크래시면 재시작한 로봇이 그 잠금 요청에 RELEASE 를 보낸다.       *)
(*                                                                         *)
(* Mutation                                                                *)
(*   "none"              설계                                              *)
(*   "forget"            재시작 때 잠금 표를 잊는다                         *)
(*   "expireCommitted"   기한 만료가 확정 잠금도 회수한다                   *)
(*   "releaseAfterDone"  완료 후 잠금을 풀어 준다 (묘비 없음)               *)
(***************************************************************************)
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS Robots, Tasks, NoRid, MaxReqs, MaxCrashes, Mutation

ASSUME MaxReqs \in Nat /\ MaxCrashes \in Nat
ASSUME Mutation \in {"none", "forget", "expireCommitted", "releaseAfterDone"}

N == Cardinality(Robots)
Quorum == N \div 2 + 1

Free == [st |-> "F", rid |-> NoRid]
T(rid) == [st |-> "T", rid |-> rid]
C(rid) == [st |-> "C", rid |-> rid]

ReqM(rid, t) == [type |-> "req", rid |-> rid, task |-> t]
VoteM(rid, v) == [type |-> "vote", rid |-> rid, from |-> v]
ConfirmM(rid, t) == [type |-> "confirm", rid |-> rid, task |-> t]
AckM(rid, v) == [type |-> "ack", rid |-> rid, from |-> v]
AbandonM(rid) == [type |-> "abandon", rid |-> rid]
ReleaseM(rid) == [type |-> "release", rid |-> rid]

VARIABLES lock,      \* [Robots -> [Tasks -> 표]]        (영속)
          phase,     \* [Robots -> {"idle","locking","confirming","locked","started"}]
          cur,       \* [Robots -> 요청 id | NoRid]
          curT,      \* [Robots -> 작업 | NoRid]
          inc,       \* [Robots -> incarnation]          (영속)
          seq,       \* [Robots -> 일련번호]             (휘발)
          msgs,      \* 보낸 메시지 전부 (지우지 않는다)
          execCount, \* [Tasks -> 픽업 시작 횟수]        (역사 변수)
          nreqs, ncrash

vars == <<lock, phase, cur, curT, inc, seq, msgs, execCount, nreqs, ncrash>>

Rid == Robots \X (0..MaxCrashes) \X (1..MaxReqs)

TypeOK ==
  /\ lock \in [Robots -> [Tasks -> {Free} \cup [st : {"T", "C"}, rid : Rid]]]
  /\ phase \in [Robots -> {"idle", "locking", "confirming", "locked", "started"}]
  /\ cur \in [Robots -> Rid \cup {NoRid}]
  /\ curT \in [Robots -> Tasks \cup {NoRid}]
  /\ inc \in [Robots -> 0..MaxCrashes]
  /\ seq \in [Robots -> 0..MaxReqs]
  /\ execCount \in [Tasks -> 0..MaxReqs]
  /\ nreqs \in 0..MaxReqs
  /\ ncrash \in 0..MaxCrashes

Init ==
  /\ lock = [v \in Robots |-> [t \in Tasks |-> Free]]
  /\ phase = [r \in Robots |-> "idle"]
  /\ cur = [r \in Robots |-> NoRid]
  /\ curT = [r \in Robots |-> NoRid]
  /\ inc = [r \in Robots |-> 0]
  /\ seq = [r \in Robots |-> 0]
  /\ msgs = {}
  /\ execCount = [t \in Tasks |-> 0]
  /\ nreqs = 0
  /\ ncrash = 0

Senders(type, rid) == {m.from : m \in {mm \in msgs : mm.type = type /\ mm.rid = rid}}

ToIdle(r) ==
  /\ phase' = [phase EXCEPT ![r] = "idle"]
  /\ cur' = [cur EXCEPT ![r] = NoRid]
  /\ curT' = [curT EXCEPT ![r] = NoRid]

(* 요청자 ------------------------------------------------------------------ *)
Request(r) ==
  /\ phase[r] = "idle"
  /\ nreqs < MaxReqs
  /\ \E t \in Tasks :
       LET rid == <<r, inc[r], seq[r] + 1>> IN
         /\ cur' = [cur EXCEPT ![r] = rid]
         /\ curT' = [curT EXCEPT ![r] = t]
         /\ msgs' = msgs \cup {ReqM(rid, t)}
  /\ seq' = [seq EXCEPT ![r] = @ + 1]
  /\ phase' = [phase EXCEPT ![r] = "locking"]
  /\ nreqs' = nreqs + 1
  /\ UNCHANGED <<lock, inc, execCount, ncrash>>

SendConfirm(r) ==
  /\ phase[r] = "locking"
  /\ Cardinality(Senders("vote", cur[r])) >= Quorum
  /\ phase' = [phase EXCEPT ![r] = "confirming"]
  /\ msgs' = msgs \cup {ConfirmM(cur[r], curT[r])}
  /\ UNCHANGED <<lock, cur, curT, inc, seq, execCount, nreqs, ncrash>>

Locked(r) ==
  /\ phase[r] = "confirming"
  /\ Cardinality(Senders("ack", cur[r])) >= Quorum
  /\ phase' = [phase EXCEPT ![r] = "locked"]
  /\ UNCHANGED <<lock, cur, curT, inc, seq, msgs, execCount, nreqs, ncrash>>

(* 픽업 시작 -- 되돌릴 수 없다. 중복 실행을 세는 곳. *)
Start(r) ==
  /\ phase[r] = "locked"
  /\ phase' = [phase EXCEPT ![r] = "started"]
  /\ execCount' = [execCount EXCEPT ![curT[r]] = @ + 1]
  /\ UNCHANGED <<lock, cur, curT, inc, seq, msgs, nreqs, ncrash>>

(* 완료. 설계는 잠금을 묘비로 남긴다. releaseAfterDone 변이는 푼다. *)
Done(r) ==
  /\ phase[r] = "started"
  /\ ToIdle(r)
  /\ msgs' = IF Mutation = "releaseAfterDone" THEN msgs \cup {ReleaseM(cur[r])} ELSE msgs
  /\ UNCHANGED <<lock, inc, seq, execCount, nreqs, ncrash>>

(* 과반을 못 받음 -> 포기 *)
GiveUp(r) ==
  /\ phase[r] \in {"locking", "confirming"}
  /\ ToIdle(r)
  /\ msgs' = msgs \cup {AbandonM(cur[r])}
  /\ UNCHANGED <<lock, inc, seq, execCount, nreqs, ncrash>>

(* 시작 전에 내려놓음 (협상이 다른 로봇에게 줌) -> RELEASE *)
Drop(r) ==
  /\ phase[r] = "locked"
  /\ ToIdle(r)
  /\ msgs' = msgs \cup {ReleaseM(cur[r])}
  /\ UNCHANGED <<lock, inc, seq, execCount, nreqs, ncrash>>

CrashRestart(r) ==
  /\ ncrash < MaxCrashes
  /\ ncrash' = ncrash + 1
  /\ inc' = [inc EXCEPT ![r] = @ + 1]
  /\ seq' = [seq EXCEPT ![r] = 0]
  /\ ToIdle(r)
  /\ msgs' = CASE phase[r] \in {"locking", "confirming"} -> msgs \cup {AbandonM(cur[r])}
               [] phase[r] = "locked" -> msgs \cup {ReleaseM(cur[r])}
               [] OTHER -> msgs     \* idle, 또는 started -> FAILED 묘비
  /\ lock' = IF Mutation = "forget"
             THEN [lock EXCEPT ![r] = [t \in Tasks |-> Free]]
             ELSE lock
  /\ UNCHANGED <<execCount, nreqs>>

(* 투표자 ------------------------------------------------------------------ *)
GrantVote(v) ==
  \E m \in msgs :
    /\ m.type = "req"
    /\ lock[v][m.task] = Free
    /\ lock' = [lock EXCEPT ![v][m.task] = T(m.rid)]
    /\ msgs' = msgs \cup {VoteM(m.rid, v)}
    /\ UNCHANGED <<phase, cur, curT, inc, seq, execCount, nreqs, ncrash>>

CommitVote(v) ==
  \E m \in msgs :
    /\ m.type = "confirm"
    /\ lock[v][m.task] = T(m.rid)
    /\ lock' = [lock EXCEPT ![v][m.task] = C(m.rid)]
    /\ msgs' = msgs \cup {AckM(m.rid, v)}
    /\ UNCHANGED <<phase, cur, curT, inc, seq, execCount, nreqs, ncrash>>

ExpirableSt == IF Mutation = "expireCommitted" THEN {"T", "C"} ELSE {"T"}

ExpireVote(v) ==
  \E t \in Tasks :
    /\ lock[v][t].st \in ExpirableSt
    /\ lock' = [lock EXCEPT ![v][t] = Free]
    /\ UNCHANGED <<phase, cur, curT, inc, seq, msgs, execCount, nreqs, ncrash>>

FreeRid(v, rid, sts) ==
  /\ \E t \in Tasks : lock[v][t].rid = rid /\ lock[v][t].st \in sts
  /\ lock' = [lock EXCEPT ![v] = [t \in Tasks |->
               IF lock[v][t].rid = rid /\ lock[v][t].st \in sts THEN Free ELSE lock[v][t]]]

(* ABANDON 은 시작 전 요청에만 오므로 C 도 푼다 (QuorumVote 의 fixA 와 같음) *)
HandleAbandon(v) ==
  \E m \in msgs :
    /\ m.type = "abandon"
    /\ FreeRid(v, m.rid, {"T", "C"})
    /\ UNCHANGED <<phase, cur, curT, inc, seq, msgs, execCount, nreqs, ncrash>>

HandleRelease(v) ==
  \E m \in msgs :
    /\ m.type = "release"
    /\ FreeRid(v, m.rid, {"T", "C"})
    /\ UNCHANGED <<phase, cur, curT, inc, seq, msgs, execCount, nreqs, ncrash>>

Next ==
  \E r \in Robots :
    \/ Request(r) \/ SendConfirm(r) \/ Locked(r) \/ Start(r) \/ Done(r)
    \/ GiveUp(r) \/ Drop(r) \/ CrashRestart(r)
    \/ GrantVote(r) \/ CommitVote(r) \/ ExpireVote(r)
    \/ HandleAbandon(r) \/ HandleRelease(r)

Spec == Init /\ [][Next]_vars

(* 안전: 어떤 작업도 두 번 이상 픽업이 시작되지 않는다 (같은 로봇이든 아니든) *)
AtMostOnce == \A t \in Tasks : execCount[t] <= 1

Symm == Permutations(Robots) \cup Permutations(Tasks)

=============================================================================
