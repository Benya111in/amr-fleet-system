----------------------------- MODULE QuorumVote -----------------------------
(***************************************************************************)
(* 정적 과반 배타 투표 + 2단계 진입 (구역 상호배제).                       *)
(*                                                                         *)
(* 출처: 보고서 「분산 플릿 관리 협상 할당과 폴백」 82-98행의 전이 표와    *)
(* 요청자 규칙, 100-102행의 나머지 규칙(영속 투표, incarnation, 사슬 S 의   *)
(* 전부-아니면-전무 부여).                                                 *)
(*                                                                         *)
(* 로봇 r 은 요청자이자 투표자다. 요청 id 는 <<r, incarnation, seq>>.      *)
(*                                                                         *)
(* 투표자 v, 구역 z 의 표 vote[v][z]:                                      *)
(*   F (FREE) | T(rid) (TENTATIVE) | C(rid) (COMMITTED)                    *)
(*   F -> T(rid)   REQUEST(rid,S) 수신, S 의 모든 구역 표가 F               *)
(*   T(rid) -> F   기한 만료(비결정 사건 -- 어떤 시계 관계든) 또는          *)
(*                 ABANDON(rid)/RELEASE(rid) 수신                           *)
(*   T(rid) -> C(rid)  ENTER(rid,S) 수신 시 S 전체가 여전히 T(rid)          *)
(*   C(rid) -> F   RELEASE(rid) 수신 때만 (fail-closed)                     *)
(* 요청자: 과반 VOTE -> ENTER 송신 -> 과반 ENTER_ACK(rid) 뒤에만 진입.      *)
(*         기한 안에 못 받으면 ABANDON. 진출하면 RELEASE.                   *)
(*                                                                         *)
(* 네트워크: 메시지 집합 msgs 에서 절대 지우지 않는다(Paxos.tla 와 같은     *)
(* 방식). 수신은 비결정 선택이라 유실(영영 안 받음)·중복(여러 번 받음)·     *)
(* 재정렬·임의 지연이 모두 들어간다. 안전 성질에 대해서는 이것으로 충분하다. *)
(*                                                                         *)
(* 고장: CrashRestart(r) 는 크래시와 재시작을 한 단계로 묶는다 (내려가 있는 *)
(* 동안은 아무 동작도 안 하는 것과 같으므로 안전에 대해 일반성을 잃지 않음). *)
(*   영속: vote[r][*], inc[r], done[r], 진입 사실(phase=inside 와 cur/curS)  *)
(*         그리고 진행 중 요청 id (cur) -- 재시작하면 거기에 ABANDON 을 보낸다 *)
(*   휘발: seq[r] (0 으로), 진행 중 요청 (inside 가 아니면 idle 로)          *)
(* idle 이 되면 cur/curS 를 비운다 (안 쓰는 값이 상태를 가르지 않게).       *)
(* 비잔틴 고장 없음. 시계·기한 값은 모델 밖 (Expire 가 아무 때나 일어난다).  *)
(*                                                                         *)
(* Mutation: 설계의 가드를 하나씩 되돌린 변이. "none" 이 보고서 설계.       *)
(*   "forget"            재시작 때 투표를 잊는다 (영속 없음)                *)
(*   "expireCommitted"   기한 만료가 C 표도 회수한다 (리스형 fail-open)     *)
(*   "smallQuorum"       정족수 = N div 2 (과반 미만)                       *)
(*   "onePhase"          과반 VOTE 만으로 진입 (ENTER/ACK 단계 없음)        *)
(*   "noRidCheck"        ENTER_ACK 를 요청 id 가 아니라 로봇으로만 센다     *)
(*   "volatileInc"       재시작해도 incarnation 을 올리지 않는다 (id 재사용) *)
(*   "overwriteTentative" 투표자가 남의 T 표를 덮어쓴다 (C 가 아니면 부여)   *)
(*                                                                         *)
(* Variant: ABANDON 처리 (활성 결함 확인용, 안전과 별개)                    *)
(*   "report" 보고서 표 그대로: ABANDON 은 T 만 회수                         *)
(*   "fixA"   ABANDON 이 C 도 회수                                          *)
(*   "fixB"   fixA + 투표자가 이미 끝난(RELEASE/ABANDON 처리한) 요청 id 의  *)
(*            REQUEST/ENTER 를 거부 (done 집합, 영속)                        *)
(***************************************************************************)
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS Robots,      \* 로봇 = 투표자 (모델 값 집합)
          Zones,       \* 구역 (모델 값 집합)
          NoRid,       \* "요청 없음" 모델 값
          MaxReqs,     \* 전체 요청 수 상한
          MaxCrashes,  \* 전체 크래시-재시작 수 상한
          Mutation,
          Variant,
          TrackDone    \* done 집합을 기록할지 (fixB 이거나 NoStrandedCommit 을 볼 때만 TRUE)

ASSUME MaxReqs \in Nat /\ MaxCrashes \in Nat
ASSUME Mutation \in {"none", "forget", "expireCommitted", "smallQuorum",
                     "onePhase", "noRidCheck", "volatileInc",
                     "overwriteTentative"}
ASSUME Variant \in {"report", "fixA", "fixB"}
ASSUME TrackDone \in BOOLEAN /\ (Variant = "fixB" => TrackDone)

N == Cardinality(Robots)
Quorum == IF Mutation = "smallQuorum" THEN N \div 2 ELSE N \div 2 + 1

Chains == (SUBSET Zones) \ {{}}

Free == [st |-> "F", rid |-> NoRid]
T(rid) == [st |-> "T", rid |-> rid]
C(rid) == [st |-> "C", rid |-> rid]

ReqM(rid, S) == [type |-> "req", rid |-> rid, zs |-> S]
VoteM(rid, v) == [type |-> "vote", rid |-> rid, from |-> v]
EnterM(rid, S) == [type |-> "enter", rid |-> rid, zs |-> S]
AckM(rid, v) == [type |-> "ack", rid |-> rid, from |-> v]
AbandonM(rid) == [type |-> "abandon", rid |-> rid]
ReleaseM(rid) == [type |-> "release", rid |-> rid]

VARIABLES vote,    \* [Robots -> [Zones -> 표]]            (영속)
          done,    \* [Robots -> 끝난 요청 id 집합]        (영속, fixB 에서만 거부에 씀)
          phase,   \* [Robots -> {"idle","voting","entering","inside"}]
          cur,     \* [Robots -> 지금 요청 id | NoRid]     (영속)
          curS,    \* [Robots -> 지금 요청의 구역 사슬]    (영속)
          inc,     \* [Robots -> incarnation]              (영속)
          seq,     \* [Robots -> 일련번호]                 (휘발)
          msgs,    \* 보낸 메시지 전부 (지우지 않는다)
          nreqs,   \* 전체 요청 수 (범위 제한용)
          ncrash   \* 전체 크래시 수 (범위 제한용)

vars == <<vote, done, phase, cur, curS, inc, seq, msgs, nreqs, ncrash>>

Rid == Robots \X (0..MaxCrashes) \X (1..MaxReqs)

TypeOK ==
  /\ vote \in [Robots -> [Zones -> {Free} \cup [st : {"T", "C"}, rid : Rid]]]
  /\ done \in [Robots -> SUBSET Rid]
  /\ phase \in [Robots -> {"idle", "voting", "entering", "inside"}]
  /\ cur \in [Robots -> Rid \cup {NoRid}]
  /\ curS \in [Robots -> SUBSET Zones]
  /\ inc \in [Robots -> 0..MaxCrashes]
  /\ seq \in [Robots -> 0..MaxReqs]
  /\ nreqs \in 0..MaxReqs
  /\ ncrash \in 0..MaxCrashes

Init ==
  /\ vote = [v \in Robots |-> [z \in Zones |-> Free]]
  /\ done = [v \in Robots |-> {}]
  /\ phase = [r \in Robots |-> "idle"]
  /\ cur = [r \in Robots |-> NoRid]
  /\ curS = [r \in Robots |-> {}]
  /\ inc = [r \in Robots |-> 0]
  /\ seq = [r \in Robots |-> 0]
  /\ msgs = {}
  /\ nreqs = 0
  /\ ncrash = 0

(* 요청 id rid 에 대해 type 메시지를 보낸 투표자 집합 *)
Senders(type, rid) == {m.from : m \in {mm \in msgs : mm.type = type /\ mm.rid = rid}}

(* ENTER_ACK 를 세는 집합. noRidCheck 변이는 같은 로봇의 아무 요청 ACK 나 센다. *)
AckSenders(r) ==
  IF Mutation = "noRidCheck"
  THEN {m.from : m \in {mm \in msgs : mm.type = "ack" /\ mm.rid[1] = r}}
  ELSE Senders("ack", cur[r])

(***************************************************************************)
(* 요청자                                                                   *)
(***************************************************************************)
Request(r) ==
  /\ phase[r] = "idle"
  /\ nreqs < MaxReqs
  /\ \E S \in Chains :
       LET rid == <<r, inc[r], seq[r] + 1>> IN
         /\ cur' = [cur EXCEPT ![r] = rid]
         /\ curS' = [curS EXCEPT ![r] = S]
         /\ msgs' = msgs \cup {ReqM(rid, S)}
  /\ seq' = [seq EXCEPT ![r] = @ + 1]
  /\ phase' = [phase EXCEPT ![r] = "voting"]
  /\ nreqs' = nreqs + 1
  /\ UNCHANGED <<vote, done, inc, ncrash>>

(* 과반 VOTE -> ENTER 송신 *)
SendEnter(r) ==
  /\ Mutation /= "onePhase"
  /\ phase[r] = "voting"
  /\ Cardinality(Senders("vote", cur[r])) >= Quorum
  /\ phase' = [phase EXCEPT ![r] = "entering"]
  /\ msgs' = msgs \cup {EnterM(cur[r], curS[r])}
  /\ UNCHANGED <<vote, done, cur, curS, inc, seq, nreqs, ncrash>>

(* 과반 ENTER_ACK -> 진입선 통과 *)
EnterZone(r) ==
  /\ Mutation /= "onePhase"
  /\ phase[r] = "entering"
  /\ Cardinality(AckSenders(r)) >= Quorum
  /\ phase' = [phase EXCEPT ![r] = "inside"]
  /\ UNCHANGED <<vote, done, cur, curS, inc, seq, msgs, nreqs, ncrash>>

(* onePhase 변이: 과반 VOTE(잠정 표)만으로 진입 *)
EnterOnePhase(r) ==
  /\ Mutation = "onePhase"
  /\ phase[r] = "voting"
  /\ Cardinality(Senders("vote", cur[r])) >= Quorum
  /\ phase' = [phase EXCEPT ![r] = "inside"]
  /\ UNCHANGED <<vote, done, cur, curS, inc, seq, msgs, nreqs, ncrash>>

(* 기한 안에 과반을 못 받음 -> 진입하지 않고 ABANDON *)
GiveUp(r) ==
  /\ phase[r] \in {"voting", "entering"}
  /\ phase' = [phase EXCEPT ![r] = "idle"]
  /\ msgs' = msgs \cup {AbandonM(cur[r])}
  /\ cur' = [cur EXCEPT ![r] = NoRid]
  /\ curS' = [curS EXCEPT ![r] = {}]
  /\ UNCHANGED <<vote, done, inc, seq, nreqs, ncrash>>

(* 진출 -> RELEASE *)
Exit(r) ==
  /\ phase[r] = "inside"
  /\ phase' = [phase EXCEPT ![r] = "idle"]
  /\ msgs' = msgs \cup {ReleaseM(cur[r])}
  /\ cur' = [cur EXCEPT ![r] = NoRid]
  /\ curS' = [curS EXCEPT ![r] = {}]
  /\ UNCHANGED <<vote, done, inc, seq, nreqs, ncrash>>

(***************************************************************************)
(* 크래시-재시작 (요청자와 투표자가 같은 프로세스)                          *)
(***************************************************************************)
CrashRestart(r) ==
  /\ ncrash < MaxCrashes
  /\ ncrash' = ncrash + 1
  /\ inc' = IF Mutation = "volatileInc" THEN inc ELSE [inc EXCEPT ![r] = @ + 1]
  /\ seq' = [seq EXCEPT ![r] = 0]
  /\ IF phase[r] = "inside"
     THEN UNCHANGED <<phase, msgs, cur, curS>>  \* 몸체는 구역 안에 그대로, 진입 사실은 영속
     ELSE /\ phase' = [phase EXCEPT ![r] = "idle"]
          /\ msgs' = IF phase[r] \in {"voting", "entering"}
                     THEN msgs \cup {AbandonM(cur[r])}
                     ELSE msgs
          /\ cur' = [cur EXCEPT ![r] = NoRid]
          /\ curS' = [curS EXCEPT ![r] = {}]
  /\ IF Mutation = "forget"
     THEN /\ vote' = [vote EXCEPT ![r] = [z \in Zones |-> Free]]
          /\ done' = [done EXCEPT ![r] = {}]
     ELSE UNCHANGED <<vote, done>>
  /\ UNCHANGED <<nreqs>>

(***************************************************************************)
(* 투표자                                                                   *)
(***************************************************************************)
Refused(v, rid) == Variant = "fixB" /\ rid \in done[v]

CanGrant(v, z) ==
  IF Mutation = "overwriteTentative"
  THEN vote[v][z].st /= "C"
  ELSE vote[v][z].st = "F"

(* REQUEST(rid,S): S 전체가 비어 있으면 S 전체에 원자적으로 잠정 표 *)
GrantVote(v) ==
  \E m \in msgs :
    /\ m.type = "req"
    /\ ~Refused(v, m.rid)
    /\ \A z \in m.zs : CanGrant(v, z)
    /\ \E z \in m.zs : vote[v][z] /= T(m.rid)
    /\ vote' = [vote EXCEPT ![v] = [z \in Zones |->
                  IF z \in m.zs THEN T(m.rid) ELSE vote[v][z]]]
    /\ msgs' = msgs \cup {VoteM(m.rid, v)}
    /\ UNCHANGED <<done, phase, cur, curS, inc, seq, nreqs, ncrash>>

(* ENTER(rid,S): S 전체가 여전히 T(rid) 이면 확정하고 ACK *)
CommitVote(v) ==
  \E m \in msgs :
    /\ m.type = "enter"
    /\ ~Refused(v, m.rid)
    /\ \A z \in m.zs : vote[v][z] = T(m.rid)
    /\ vote' = [vote EXCEPT ![v] = [z \in Zones |->
                  IF z \in m.zs THEN C(m.rid) ELSE vote[v][z]]]
    /\ msgs' = msgs \cup {AckM(m.rid, v)}
    /\ UNCHANGED <<done, phase, cur, curS, inc, seq, nreqs, ncrash>>

ExpirableSt == IF Mutation = "expireCommitted" THEN {"T", "C"} ELSE {"T"}

(* 기한 만료: 투표자 혼자, 아무 때나 (어떤 시계 관계에서도 안전한지 본다) *)
ExpireVote(v) ==
  \E z0 \in Zones :
    /\ vote[v][z0].st \in ExpirableSt
    /\ LET rid == vote[v][z0].rid IN
         vote' = [vote EXCEPT ![v] = [z \in Zones |->
                    IF vote[v][z].rid = rid /\ vote[v][z].st \in ExpirableSt
                    THEN Free ELSE vote[v][z]]]
    /\ UNCHANGED <<done, phase, cur, curS, inc, seq, msgs, nreqs, ncrash>>

AbandonFrees == IF Variant = "report" THEN {"T"} ELSE {"T", "C"}

(* ABANDON(rid) / RELEASE(rid) 처리. done 은 처리 기록 (fixB 에서 거부에 쓴다). *)
Finish(v, rid, freeSt) ==
  /\ vote' = [vote EXCEPT ![v] = [z \in Zones |->
                IF vote[v][z].rid = rid /\ vote[v][z].st \in freeSt
                THEN Free ELSE vote[v][z]]]
  /\ done' = IF TrackDone THEN [done EXCEPT ![v] = @ \cup {rid}] ELSE done

HandleAbandon(v) ==
  \E m \in msgs :
    /\ m.type = "abandon"
    /\ (m.rid \notin done[v] \/ \E z \in Zones :
          vote[v][z].rid = m.rid /\ vote[v][z].st \in AbandonFrees)
    /\ Finish(v, m.rid, AbandonFrees)
    /\ UNCHANGED <<phase, cur, curS, inc, seq, msgs, nreqs, ncrash>>

HandleRelease(v) ==
  \E m \in msgs :
    /\ m.type = "release"
    /\ (m.rid \notin done[v] \/ \E z \in Zones : vote[v][z].rid = m.rid)
    /\ Finish(v, m.rid, {"T", "C"})
    /\ UNCHANGED <<phase, cur, curS, inc, seq, msgs, nreqs, ncrash>>

Next ==
  \E r \in Robots :
    \/ Request(r)
    \/ SendEnter(r)
    \/ EnterZone(r)
    \/ EnterOnePhase(r)
    \/ GiveUp(r)
    \/ Exit(r)
    \/ CrashRestart(r)
    \/ GrantVote(r)
    \/ CommitVote(r)
    \/ ExpireVote(r)
    \/ HandleAbandon(r)
    \/ HandleRelease(r)

Spec == Init /\ [][Next]_vars

(***************************************************************************)
(* 성질                                                                     *)
(***************************************************************************)
Inside(r) == phase[r] = "inside"

(* 안전: 모든 구역에서 안에 있는 로봇 <= 1 *)
MutualExclusion ==
  \A z \in Zones : Cardinality({r \in Robots : Inside(r) /\ z \in curS[r]}) <= 1

(* 보조: 안에 있는 로봇은 과반 투표자의 C(rid) 를 S 전체에 대해 쥐고 있다 *)
InsideBacked ==
  \A r \in Robots : Inside(r) =>
    Cardinality({v \in Robots : \A z \in curS[r] : vote[v][z] = C(cur[r])}) >= Quorum

(* 활성 결함 탐지 (안전 아님): 아무 후속 실행에서도 풀릴 수 없는 C 표.     *)
(* C(rid) 를 푸는 것은 RELEASE(rid) (그리고 fixA/B 에서는 ABANDON(rid)) 뿐. *)
(* 요청자가 rid 로 진입 중/안에 있지 않고, v 가 아직 처리하지 않은 해제     *)
(* 메시지도 없으면 그 표는 영원히 남는다 (cur 은 단조로 앞으로만 간다).      *)
PendingFree(v, rid) ==
  /\ rid \notin done[v]
  /\ \/ ReleaseM(rid) \in msgs
     \/ (Variant /= "report" /\ AbandonM(rid) \in msgs)

Stranded(v, z) ==
  /\ vote[v][z].st = "C"
  /\ LET rid == vote[v][z].rid
         r == rid[1]
     IN /\ ~(cur[r] = rid /\ phase[r] \in {"entering", "inside"})
        /\ ~PendingFree(v, rid)

NoStrandedCommit == \A v \in Robots, z \in Zones : ~Stranded(v, z)

Symm == Permutations(Robots) \cup Permutations(Zones)

=============================================================================
