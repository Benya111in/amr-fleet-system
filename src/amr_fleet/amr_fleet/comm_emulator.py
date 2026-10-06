"""
앱 수준 통신 모의 CommEmulator — 링크별 지연·손실·분할 (보너스 4 P0).

근거: 연구 보고서 「분산 플릿 관리 협상 할당과 폴백」 고장 주입 절. 이 서버에서는 OS 수준 주입(tc netem)이
커널 모듈이 없어 안 되고, 같은 호스트의 DDS 는 SHM 으로 가므로 loopback 주입도 데이터에 닿지 않는다.
그래서 노드가 보내기(또는 받기) 전에 이 모의를 거친다. 설계: docs/research/fleet-distributed/p0.md §1.

링크 = (송신자, 수신자) 이름 쌍이다. 이름은 'fleet'(중앙: fleet_manager · traffic_manager), 로봇 id
('amr_01' …)이고, 로봇↔로봇 링크도 같은 방식으로 다룬다. 설정의 src/dst 는 fnmatch 패턴('amr_*', '*')이다.

링크마다:
- 지연: U(min_ms, max_ms) ms.
- 손실: Bernoulli(drop_rate) 와 Gilbert–Elliott 버스트(선택)를 함께 건다 (둘 중 하나라도 잃으면 유실).
  버스트 상태열은 **sim 시각의 함수**다 — 좋은/나쁜 상태의 체류 시간을 지수 분포로 미리 정한다.
  메시지를 몇 개 보내든 버스트가 오는 시각은 같다 (정책이 메시지를 더 보내도 외생 입력이 바뀌지 않는다).
- 분할 일정 Partition: [start_s, end_s) 동안 서로 다른 그룹 사이 메시지를 모두 잃는다 (대칭).
  그룹에 적지 않은 이름은 "나머지" 한 그룹이다.
- 단절 일정 Outage: [start_s, end_s) 동안 src→dst 방향만 잃는다 (비대칭 분할, 상향 링크만 끊김 등).

결정성 (같은 시드 → 같은 지연열): 링크의 k 번째 메시지가 쓰는 난수는 (루트 시드, src, dst, k) 의 해시에서,
버스트 상태열은 (루트 시드, src, dst, 구간 번호) 의 해시에서 나온다. 다른 링크의 메시지 수·순서와 무관하다
(공통 난수, NumPy SeedSequence 와 같은 생각). seed=0 은 "무작위" 지만 실효 시드를 OS 엔트로피로 하나 뽑아
기록하므로(effective seed ≠ 0) 그 실행도 나중에 같은 시드로 재현할 수 있다.

시각은 호출자가 넘기는 노드 시계 초다 (use_sim_time 이면 /clock). 이 모듈은 벽시계를 읽지 않는다.
rclpy 비의존 순수 모듈이다 (pytest 로 바로 시험한다).
"""

from __future__ import annotations

import bisect
import collections
import dataclasses
import fnmatch
import hashlib
import math
import secrets
from typing import Any, Deque, Dict, FrozenSet, List, Mapping, Optional, Sequence, Tuple

from amr_fleet.latency import SPEC_MAX_LATENCY_MS, parse_latency_range

FLEET = 'fleet'            # 중앙 쪽 링크 끝 이름 (fleet_manager · traffic_manager)
_U64 = float(2 ** 64)

# 사건 이유 (history · stats)
DELIVERED = 'delivered'
DROP_LOSS = 'loss'         # Bernoulli drop_rate
DROP_BURST = 'burst'       # Gilbert–Elliott 나쁜/좋은 상태 손실
DROP_PARTITION = 'partition'
DROP_OUTAGE = 'outage'


def resolve_seed(seed: int) -> int:
    """실효 루트 시드. 0 이면 OS 엔트로피에서 0 이 아닌 63 비트 값을 하나 뽑는다."""
    seed = int(seed)
    if seed < 0:
        raise ValueError(f'seed 는 0 이상이어야 한다: {seed}')
    while seed == 0:
        seed = secrets.randbits(63)
    return seed


def _uniforms(seed: int, *key: Any) -> Tuple[float, float, float]:
    """(seed, key…) → [0, 1) 균등 난수 3 개. blake2b — 파이썬 버전·프로세스와 무관하게 같은 값."""
    text = '\x1f'.join([str(seed)] + [str(k) for k in key]).encode('utf-8')
    d = hashlib.blake2b(text, digest_size=24).digest()
    return tuple(int.from_bytes(d[i:i + 8], 'little') / _U64 for i in (0, 8, 16))


@dataclasses.dataclass(frozen=True)
class GilbertElliott:
    """
    2 상태 버스트 손실. 체류 시간은 sim 초의 지수 분포.

    mean_good_s / mean_bad_s: 좋은 · 나쁜 상태 평균 체류 [s]. loss_good / loss_bad: 그 상태의 손실 확률.
    장기 평균 손실 = (mean_good·loss_good + mean_bad·loss_bad) / (mean_good + mean_bad).
    """

    mean_good_s: float
    mean_bad_s: float
    loss_good: float = 0.0
    loss_bad: float = 1.0

    def __post_init__(self):
        """값 검사."""
        if not (self.mean_good_s > 0.0 and self.mean_bad_s > 0.0
                and math.isfinite(self.mean_good_s) and math.isfinite(self.mean_bad_s)):
            raise ValueError(f'버스트 체류 시간은 양의 유한값이어야 한다: {self}')
        for p in (self.loss_good, self.loss_bad):
            if not 0.0 <= p <= 1.0:
                raise ValueError(f'버스트 손실 확률은 [0, 1] 이어야 한다: {self}')

    @property
    def mean_loss(self) -> float:
        """장기 평균 손실률."""
        g, b = self.mean_good_s, self.mean_bad_s
        return (g * self.loss_good + b * self.loss_bad) / (g + b)


@dataclasses.dataclass(frozen=True)
class LinkModel:
    """한 링크의 지연 구간 [ms], Bernoulli 손실, 버스트(선택)."""

    min_ms: float = 0.0
    max_ms: float = SPEC_MAX_LATENCY_MS
    drop_rate: float = 0.0
    burst: Optional[GilbertElliott] = None

    def __post_init__(self):
        """값 검사."""
        parse_latency_range([self.min_ms, self.max_ms])
        if not 0.0 <= self.drop_rate < 1.0:
            raise ValueError(f'drop_rate 는 [0, 1) 이어야 한다: {self.drop_rate}')

    @property
    def lossy(self) -> bool:
        """손실이 생길 수 있나."""
        b = self.burst
        return self.drop_rate > 0.0 or (b is not None and max(b.loss_good, b.loss_bad) > 0.0)

    @property
    def active(self) -> bool:
        """지연 또는 손실이 실제로 걸리나 (아니면 호출자가 큐 없이 바로 보낸다)."""
        return self.max_ms > 0.0 or self.lossy

    def describe(self) -> str:
        """로그용 한 줄."""
        text = f'U[{self.min_ms:g}, {self.max_ms:g}] ms, drop={self.drop_rate:g}'
        if self.burst is not None:
            b = self.burst
            text += (f', burst(good {b.mean_good_s:g} s/{b.loss_good:g}, '
                     f'bad {b.mean_bad_s:g} s/{b.loss_bad:g})')
        return text


@dataclasses.dataclass(frozen=True)
class Partition:
    """[start_s, end_s) 동안 서로 다른 그룹 사이 메시지 유실. 적지 않은 이름은 '나머지' 그룹."""

    start_s: float
    end_s: float
    groups: Tuple[FrozenSet[str], ...]

    def __post_init__(self):
        """값 검사: 구간, 그룹이 서로소인지."""
        if not self.end_s > self.start_s:
            raise ValueError(f'분할 구간은 end_s > start_s 여야 한다: {self}')
        seen: set = set()
        for g in self.groups:
            if seen & g:
                raise ValueError(f'분할 그룹이 겹친다: {sorted(seen & g)}')
            seen |= g

    def _group(self, name: str) -> int:
        for i, g in enumerate(self.groups):
            if name in g:
                return i
        return -1

    def cuts(self, src: str, dst: str, now: float) -> bool:
        """지금 src 와 dst 가 갈라져 있나."""
        return self.start_s <= now < self.end_s and self._group(src) != self._group(dst)


@dataclasses.dataclass(frozen=True)
class Outage:
    """[start_s, end_s) 동안 src→dst 방향 유실 (fnmatch 패턴)."""

    start_s: float
    end_s: float
    src: str = '*'
    dst: str = '*'

    def __post_init__(self):
        """값 검사."""
        if not self.end_s > self.start_s:
            raise ValueError(f'단절 구간은 end_s > start_s 여야 한다: {self}')

    def cuts(self, src: str, dst: str, now: float) -> bool:
        """지금 이 방향이 끊겼나."""
        return (self.start_s <= now < self.end_s and fnmatch.fnmatchcase(src, self.src)
                and fnmatch.fnmatchcase(dst, self.dst))


@dataclasses.dataclass(frozen=True)
class LinkEvent:
    """메시지 1건의 모의 결과 (진단 · 시험용)."""

    time: float
    src: str
    dst: str
    index: int                    # 그 링크의 k (0 부터)
    delay_s: Optional[float]      # None = 유실
    outcome: str                  # DELIVERED | DROP_*


class _BurstTrack:
    """한 링크의 버스트 상태열: sim 시각 → 좋은/나쁜. 경계 시각을 필요한 만큼만 늘린다."""

    def __init__(self, seed: int, src: str, dst: str, model: GilbertElliott):
        self._seed, self._src, self._dst, self._m = seed, src, dst, model
        self._bounds: List[float] = []        # i 번째 체류 구간의 끝 시각. 0 번은 t=0 부터 좋은 상태

    def _extend(self, until: float) -> None:
        t = self._bounds[-1] if self._bounds else 0.0
        while t <= until:
            j = len(self._bounds)
            u = _uniforms(self._seed, 'ge', self._src, self._dst, j)[0]
            mean = self._m.mean_good_s if j % 2 == 0 else self._m.mean_bad_s
            t += -mean * math.log1p(-u)       # Exp(mean). u < 1 이므로 유한
            self._bounds.append(t)

    def bad(self, now: float) -> bool:
        """그 시각(now)에 나쁜 상태인가 (now < 0 은 좋은 상태)."""
        if now < 0.0:
            return False
        self._extend(now)
        return bisect.bisect_right(self._bounds, now) % 2 == 1


class CommEmulator:
    """
    링크별 지연·손실·분할 모의.

    transmit(src, dst, now) 가 그 메시지의 지연 [s] 을 돌려주고, 잃으면 None 이다. 호출마다 그 링크의 k 가
    하나 는다 (잃어도). simulate=false 면 언제나 0.0 (실기 배포 — 아무것도 주입하지 않는다).
    """

    def __init__(self, seed: int = 0, default: LinkModel = LinkModel(),
                 links: Sequence[Tuple[str, str, LinkModel]] = (),
                 partitions: Sequence[Partition] = (), outages: Sequence[Outage] = (),
                 simulate: bool = True, history: int = 1000):
        self.requested_seed = int(seed)
        self.seed = resolve_seed(seed)
        self.simulate = bool(simulate)
        self.default = default
        self.links: Tuple[Tuple[str, str, LinkModel], ...] = tuple(links)
        self.partitions: Tuple[Partition, ...] = tuple(partitions)
        self.outages: Tuple[Outage, ...] = tuple(outages)
        self._count: Dict[Tuple[str, str], int] = collections.Counter()
        self._bursts: Dict[Tuple[str, str], _BurstTrack] = {}
        self._models: Dict[Tuple[str, str], LinkModel] = {}
        self.stats: Dict[str, int] = collections.Counter()
        # 최근 사건 (진단 · 시험: 주입값이 설정 범위 안인지, 같은 시드에서 같은 열인지)
        self.history: Deque[LinkEvent] = collections.deque(maxlen=history)

    # ------------------------------------------------------------------ 설정
    @classmethod
    def from_config(cls, config: Optional[Mapping[str, Any]], seed: int = 0,
                    default: LinkModel = LinkModel(), simulate: bool = True) -> 'CommEmulator':
        """
        설정 사전(YAML 파일 내용)으로 만든다. 없는 키는 비운다.

        links:      [{src, dst, comm_latency_ms?, drop_rate?, burst?: {mean_good_s, mean_bad_s,
                     loss_good?, loss_bad?}}]  — 위에서부터 처음 맞는 항목. 빠진 값은 default 에서.
        partitions: [{start_s, end_s, groups: [[이름…], …]}]
        outages:    [{start_s, end_s, src?, dst?}]
        seed:       있으면 인자 seed 대신 쓴다 (0 이 아닐 때).
        """
        config = dict(config or {})
        unknown = set(config) - {'links', 'partitions', 'outages', 'seed'}
        if unknown:
            raise ValueError(f'모르는 통신 모의 설정 키: {sorted(unknown)}')
        links = [(str(e.get('src', '*')), str(e.get('dst', '*')), link_model(e, default))
                 for e in config.get('links') or []]
        partitions = [Partition(float(e['start_s']), float(e['end_s']), _groups(e.get('groups')))
                      for e in config.get('partitions') or []]
        outages = [Outage(float(e['start_s']), float(e['end_s']), str(e.get('src', '*')),
                          str(e.get('dst', '*')))
                   for e in config.get('outages') or []]
        file_seed = int(config.get('seed') or 0)
        return cls(seed=file_seed or seed, default=default, links=links, partitions=partitions,
                   outages=outages, simulate=simulate)

    def model(self, src: str, dst: str) -> LinkModel:
        """src→dst 링크 모델: links 에서 처음 맞는 항목, 없으면 default."""
        key = (src, dst)
        m = self._models.get(key)
        if m is None:
            m = self.default
            for psrc, pdst, lm in self.links:
                if fnmatch.fnmatchcase(src, psrc) and fnmatch.fnmatchcase(dst, pdst):
                    m = lm
                    break
            self._models[key] = m
        return m

    def enabled(self, src: str, dst: str) -> bool:
        """이 링크에 무엇이든 주입되나 (아니면 호출자가 큐 없이 바로 보낸다)."""
        if not self.simulate:
            return False
        if self.model(src, dst).active:
            return True
        return any(p.groups for p in self.partitions) or any(
            fnmatch.fnmatchcase(src, o.src) and fnmatch.fnmatchcase(dst, o.dst)
            for o in self.outages)

    def max_ms(self) -> float:
        """설정한 지연 상한 중 최댓값 [ms]."""
        return max([self.default.max_ms] + [lm.max_ms for _, _, lm in self.links])

    def exceeds_spec(self) -> bool:
        """지연 상한이 명세 최대(100 ms)를 넘는 링크가 있나 (스트레스 시험 설정 경고)."""
        return self.max_ms() > SPEC_MAX_LATENCY_MS

    def describe(self) -> str:
        """로그용 한 줄 요약 (실효 시드 포함 — 재현하려면 이 값을 seed 로 준다)."""
        if not self.simulate:
            return 'off'
        seed = f'seed={self.seed}' + (' (무작위로 뽑음)' if self.requested_seed == 0 else '')
        extra = ''
        if self.links:
            extra += f', 링크 규칙 {len(self.links)}'
        if self.partitions:
            extra += f', 분할 {len(self.partitions)}'
        if self.outages:
            extra += f', 단절 {len(self.outages)}'
        return f'{self.default.describe()}{extra}, {seed}'

    # ------------------------------------------------------------------ 모의
    def cut_reason(self, src: str, dst: str, now: float) -> Optional[str]:
        """지금 일정(분할 · 단절)으로 끊긴 링크면 그 이유, 아니면 None."""
        if any(p.cuts(src, dst, now) for p in self.partitions):
            return DROP_PARTITION
        if any(o.cuts(src, dst, now) for o in self.outages):
            return DROP_OUTAGE
        return None

    def transmit(self, src: str, dst: str, now: float) -> Optional[float]:
        """메시지 1건: 지연 [s] 또는 유실(None). 그 링크의 k 를 하나 올린다."""
        key = (str(src), str(dst))
        k = self._count[key]
        self._count[key] = k + 1
        if not self.simulate:
            self.stats[DELIVERED] += 1
            return 0.0
        m = self.model(*key)
        # 한 해시에서 세 몫: Bernoulli 손실 · 지연 · 버스트 손실 (서로 겹치지 않게 다른 몫을 쓴다)
        u_loss, u_delay, u_burst = _uniforms(self.seed, 'msg', key[0], key[1], k)
        outcome = self.cut_reason(key[0], key[1], now)
        if outcome is None and m.drop_rate > 0.0 and u_loss < m.drop_rate:
            outcome = DROP_LOSS
        if outcome is None and m.burst is not None:
            p = m.burst.loss_bad if self._burst(key, m.burst).bad(now) else m.burst.loss_good
            if u_burst < p:
                outcome = DROP_BURST
        if outcome is not None:
            self.stats[outcome] += 1
            self.history.append(LinkEvent(now, key[0], key[1], k, None, outcome))
            return None
        delay = (m.min_ms + u_delay * (m.max_ms - m.min_ms)) * 1e-3 if m.max_ms > 0.0 else 0.0
        self.stats[DELIVERED] += 1
        self.history.append(LinkEvent(now, key[0], key[1], k, delay, DELIVERED))
        return delay

    def count(self, src: str, dst: str) -> int:
        """지금까지 그 링크로 보낸 메시지 수 (다음 k)."""
        return self._count[(src, dst)]

    def burst_bad(self, src: str, dst: str, now: float) -> bool:
        """그 링크가 now 에 버스트(나쁜) 상태인가 (버스트가 없으면 False)."""
        m = self.model(src, dst)
        return m.burst is not None and self._burst((src, dst), m.burst).bad(now)

    def _burst(self, key: Tuple[str, str], model: GilbertElliott) -> _BurstTrack:
        tr = self._bursts.get(key)
        if tr is None:
            tr = self._bursts[key] = _BurstTrack(self.seed, key[0], key[1], model)
        return tr


def _groups(value: Any) -> Tuple[FrozenSet[str], ...]:
    """분할 그룹 [[이름…], …] 검사 (문자열 하나를 그룹으로 주면 글자 집합이 되므로 거절한다)."""
    def ok(g: Any) -> bool:
        return isinstance(g, (list, tuple)) and bool(g) and all(isinstance(n, str) for n in g)
    if not isinstance(value, (list, tuple)) or not all(ok(g) for g in value):
        raise ValueError(f'분할 groups 는 이름 목록의 목록이어야 한다: {value!r}')
    return tuple(frozenset(g) for g in value)


def link_model(entry: Mapping[str, Any], default: LinkModel = LinkModel()) -> LinkModel:
    """설정 항목 하나 → LinkModel. 없는 값은 default 에서 (comm_latency_ms 는 [lo, hi] 또는 숫자)."""
    lo, hi = default.min_ms, default.max_ms
    if 'comm_latency_ms' in entry:
        lo, hi = parse_latency_range(entry['comm_latency_ms'])
    burst = default.burst
    if entry.get('burst'):
        b = dict(entry['burst'])
        burst = GilbertElliott(float(b['mean_good_s']), float(b['mean_bad_s']),
                               float(b.get('loss_good', 0.0)), float(b.get('loss_bad', 1.0)))
    return LinkModel(lo, hi, float(entry.get('drop_rate', default.drop_rate)), burst)


def load_config(path: str) -> Dict[str, Any]:
    """통신 모의 YAML 파일 → 사전. 빈 경로는 빈 사전."""
    if not path:
        return {}
    import yaml   # 노드에서만 쓴다 (순수 시험은 사전을 바로 넘긴다)
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f'통신 모의 설정은 사전이어야 한다: {path}')
    return data


def from_node_params(simulate: bool, comm_latency_ms: Any, drop_rate: float, seed: int,
                     model_file: str = '') -> CommEmulator:
    """노드 파라미터(fleet.yaml 과 1:1) → CommEmulator. 기본 링크 = comm_latency_ms · drop_rate."""
    lo, hi = parse_latency_range(comm_latency_ms)
    default = LinkModel(lo, hi, float(drop_rate))
    return CommEmulator.from_config(load_config(model_file), seed=int(seed), default=default,
                                    simulate=simulate)


def summarize(events: Sequence[LinkEvent]) -> Mapping[Tuple[str, str], List[Optional[float]]]:
    """사건 → 링크별 지연열 (유실은 None). 결정성 시험·로그 비교용."""
    out: Dict[Tuple[str, str], List[Optional[float]]] = collections.defaultdict(list)
    for ev in events:
        out[(ev.src, ev.dst)].append(ev.delay_s)
    return out
