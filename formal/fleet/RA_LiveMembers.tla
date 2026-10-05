--------------------------- MODULE RA_LiveMembers ---------------------------
(***************************************************************************)
(* "생존 멤버 전원 REPLY + 멤버 >= MinMembers" Ricart-Agrawala 변형.        *)
(*                                                                         *)
(* 출처: 보고서 「분산 플릿 관리 협상 할당과 폴백」 74-80행이 기각한 초안   *)
(* (research_notes/.../decentralized_traffic.md 권장안 A, 370-372행):       *)
(*   "진입 조건: 현재 생존 멤버 전원의 REPLY 를 받았고, 그 멤버가 5대 중   *)
(*    3대 이상이다."                                                       *)
(*                                                                         *)
(* 모델 범위                                                               *)
(*  - 구역 1개. 로봇 1..N. 각 로봇은 요청자이자 응답자다.                  *)
(*  - 요청 순서는 RA 원형: (번호, id) 사전식. 번호는 지금까지 본 최댓값+1. *)
(*  - REPLY 는 RA 규칙대로: 내가 안에 있거나, 나도 요청 중이고 내가 더     *)
(*    앞서면 미룬다(deferred). 아니면 바로 보낸다.                          *)
(*  - REPLY 에는 어떤 요청에 대한 답인지(ts)를 싣고, 요청자는 지금 요청의   *)
(*    답만 센다 (초안보다 강한 쪽으로 잡았다 -- 그래도 깨지는지 보려고).   *)
(*  - 채널은 신뢰(유실 없음)·비FIFO. 반례가 메시지 유실 없이 멤버십 견해   *)
(*    불일치만으로 나온다는 것을 보이려고 일부러 유실을 넣지 않았다.        *)
(*  - view[r] 은 r 이 믿는 생존 멤버. 하트비트 오탐을 추상화해 아무 때나   *)
(*    비결정적으로 바뀐다 (ViewModel 상수로 범위를 고른다).                *)
(*                                                                         *)
(* ViewModel                                                               *)
(*  "fixed"   : view 가 늘 전원 (= RA 원형, 대조군. 위반이 없어야 한다)    *)
(*  "dropOne" : 각 로봇이 남 하나까지만 빠뜨릴 수 있다 (최소 불일치)        *)
(*  "report"  : 보고서의 장면 그대로. A(1) 는 {A,B,C}, E(N) 는 {C,D,E} 로만  *)
(*              바뀔 수 있다 (N = 5 에서 {1,2,3} / {3,4,5})                 *)
(*  "any"     : r 을 포함하고 크기 >= MinMembers 인 아무 집합               *)
(***************************************************************************)
EXTENDS Naturals, FiniteSets

CONSTANTS N,           \* 로봇 수
          MinMembers,  \* 진입에 필요한 최소 생존 멤버 수 (자기 포함)
          MaxReq,      \* 로봇당 요청 횟수 상한 (상태 공간 제한)
          ViewModel,   \* "fixed" | "dropOne" | "report" | "any"
          Requesters   \* 요청·견해 변경을 하는 로봇 (나머지는 응답만 한다)

ASSUME N \in Nat /\ N >= 2
ASSUME MinMembers \in 1..N
ASSUME MaxReq \in Nat /\ MaxReq >= 1
ASSUME ViewModel \in {"fixed", "dropOne", "report", "any"}

Robots == 1..N

ASSUME Requesters \subseteq Robots

VARIABLES state,     \* "idle" | "waiting" | "inside"
          highest,   \* 지금까지 본 가장 큰 요청 번호 (RA 의 Highest_Sequence_Number)
          reqTs,     \* 지금 요청의 번호
          replied,   \* 지금 요청에 REPLY 를 보낸 로봇 집합
          deferred,  \* 미룬 요청 <<로봇, 번호>> 집합
          view,      \* 각 로봇이 믿는 생존 멤버
          msgs,      \* 전송 중인 메시지 (수신 시 소비)
          nreq       \* 지금까지 낸 요청 수

vars == <<state, highest, reqTs, replied, deferred, view, msgs, nreq>>

Max(a, b) == IF a > b THEN a ELSE b

\* (ta, a) 가 (tb, b) 보다 앞선다
Prec(ta, a, tb, b) == ta < tb \/ (ta = tb /\ a < b)

AllowedViews(r) ==
  CASE ViewModel = "fixed"   -> {Robots}
    [] ViewModel = "dropOne" -> {Robots} \cup {Robots \ {j} : j \in Robots \ {r}}
    [] ViewModel = "report"  -> CASE r = 1 -> {Robots, {1, 2, 3}}
                                  [] r = N -> {Robots, {N - 2, N - 1, N}}
                                  [] OTHER -> {Robots}
    [] ViewModel = "any"     -> {V \in SUBSET Robots :
                                   r \in V /\ Cardinality(V) >= MinMembers}

Msg == [type : {"req", "rep"}, from : Robots, to : Robots, ts : Nat]

TypeOK ==
  /\ state \in [Robots -> {"idle", "waiting", "inside"}]
  /\ highest \in [Robots -> Nat]
  /\ reqTs \in [Robots -> Nat]
  /\ replied \in [Robots -> SUBSET Robots]
  /\ deferred \in [Robots -> SUBSET (Robots \X Nat)]
  /\ \A r \in Robots : view[r] \in AllowedViews(r)
  /\ msgs \subseteq Msg
  /\ nreq \in [Robots -> 0..MaxReq]

Init ==
  /\ state = [r \in Robots |-> "idle"]
  /\ highest = [r \in Robots |-> 0]
  /\ reqTs = [r \in Robots |-> 0]
  /\ replied = [r \in Robots |-> {}]
  /\ deferred = [r \in Robots |-> {}]
  /\ view = [r \in Robots |-> Robots]
  /\ msgs = {}
  /\ nreq = [r \in Robots |-> 0]

(* 요청: 모든 다른 로봇에게 REQUEST 를 방송한다 (생존 견해와 무관하게). *)
Request(r) ==
  /\ r \in Requesters
  /\ state[r] = "idle"
  /\ nreq[r] < MaxReq
  /\ LET t == highest[r] + 1 IN
       /\ reqTs' = [reqTs EXCEPT ![r] = t]
       /\ highest' = [highest EXCEPT ![r] = t]
       /\ msgs' = msgs \cup {[type |-> "req", from |-> r, to |-> j, ts |-> t] :
                               j \in Robots \ {r}}
  /\ state' = [state EXCEPT ![r] = "waiting"]
  /\ replied' = [replied EXCEPT ![r] = {}]
  /\ nreq' = [nreq EXCEPT ![r] = @ + 1]
  /\ UNCHANGED <<deferred, view>>

(* REQUEST 수신: RA 규칙으로 바로 답하거나 미룬다. *)
RecvRequest(j, m) ==
  /\ m \in msgs
  /\ m.type = "req"
  /\ m.to = j
  /\ highest' = [highest EXCEPT ![j] = Max(@, m.ts)]
  /\ IF \/ state[j] = "inside"
        \/ (state[j] = "waiting" /\ Prec(reqTs[j], j, m.ts, m.from))
     THEN /\ deferred' = [deferred EXCEPT ![j] = @ \cup {<<m.from, m.ts>>}]
          /\ msgs' = msgs \ {m}
     ELSE /\ msgs' = (msgs \ {m}) \cup
                     {[type |-> "rep", from |-> j, to |-> m.from, ts |-> m.ts]}
          /\ UNCHANGED deferred
  /\ UNCHANGED <<state, reqTs, replied, view, nreq>>

(* REPLY 수신: 지금 요청(ts 일치)에 대한 답만 센다. *)
RecvReply(r, m) ==
  /\ m \in msgs
  /\ m.type = "rep"
  /\ m.to = r
  /\ msgs' = msgs \ {m}
  /\ replied' = IF state[r] = "waiting" /\ m.ts = reqTs[r]
                THEN [replied EXCEPT ![r] = @ \cup {m.from}]
                ELSE replied
  /\ UNCHANGED <<state, highest, reqTs, deferred, view, nreq>>

(* 진입: "현재 생존 멤버 전원의 REPLY" + "멤버 >= MinMembers". *)
Enter(r) ==
  /\ state[r] = "waiting"
  /\ Cardinality(view[r]) >= MinMembers
  /\ view[r] \ {r} \subseteq replied[r]
  /\ state' = [state EXCEPT ![r] = "inside"]
  /\ UNCHANGED <<highest, reqTs, replied, deferred, view, msgs, nreq>>

(* 진출: 미룬 REPLY 를 모두 보낸다. *)
Exit(r) ==
  /\ state[r] = "inside"
  /\ state' = [state EXCEPT ![r] = "idle"]
  /\ msgs' = msgs \cup {[type |-> "rep", from |-> r, to |-> d[1], ts |-> d[2]] :
                          d \in deferred[r]}
  /\ deferred' = [deferred EXCEPT ![r] = {}]
  /\ UNCHANGED <<highest, reqTs, replied, view, nreq>>

(* 생존 견해 변경: 하트비트 오탐/복구의 추상화. 요청하지 않는 로봇의 견해는  *)
(* 진입에 쓰이지 않으므로 바꾸지 않는다 (상태 공간 절약, 안전 판정 불변).    *)
ChangeView(r) ==
  /\ r \in Requesters
  /\ \E V \in AllowedViews(r) \ {view[r]} :
       view' = [view EXCEPT ![r] = V]
  /\ UNCHANGED <<state, highest, reqTs, replied, deferred, msgs, nreq>>

(* 트레이스에 동작 이름이 찍히도록 메시지 선택을 감싼다. *)
DeliverRequest(j) == \E m \in msgs : RecvRequest(j, m)
DeliverReply(r) == \E m \in msgs : RecvReply(r, m)

Next ==
  \E r \in Robots :
    \/ Request(r)
    \/ Enter(r)
    \/ Exit(r)
    \/ ChangeView(r)
    \/ DeliverRequest(r)
    \/ DeliverReply(r)

Spec == Init /\ [][Next]_vars

(* 안전: 구역 안에 동시에 두 대 이상 없다. *)
MutualExclusion == Cardinality({r \in Robots : state[r] = "inside"}) <= 1

=============================================================================
