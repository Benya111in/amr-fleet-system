"""
링크 모델 — 송신자 → 수신자 메시지 1건의 유실 여부와 지연 [ms].

- PerLinkChannel     : 기본. 링크(송신자, 수신자)마다 (루트 시드, 송신자, 수신자)에서 파생한 난수열을 쓴다
                       (보고서 175행 CommEmulator 권고). 한 정책이 호출 순서를 바꿔도 다른 링크의 난수열이
                       밀리지 않는다. 손실은 Bernoulli 또는 버스트(2 상태 Gilbert 모형, 상관 rho).
- SharedStreamChannel: 재현용. 연구 계산(mrta_small.py · verify_mrta2.py)처럼 **난수 한 줄**을 송신자·수신자
                       순서대로 소비한다. 같은 시드에서 같은 유실열이 나온다.

지연은 명세 4.9 의 "최대 100 ms" 에 맞춰 U(lo, hi) ms 를 기본으로 둔다.
"""

from __future__ import annotations

import abc
import hashlib
import random
from typing import Dict, Optional, Tuple

DeliverResult = Tuple[bool, float]       # (전달됐는지, 지연 ms — 유실이면 0)


def derive_seed(*parts: object) -> int:
    """여러 값에서 64 비트 시드를 안정적으로 파생한다 (프로세스·PYTHONHASHSEED 무관)."""
    text = ':'.join(str(p) for p in parts).encode()
    return int.from_bytes(hashlib.blake2b(text, digest_size=8).digest(), 'little')


class Channel(abc.ABC):
    """메시지 1건의 유실·지연을 정한다."""

    @abc.abstractmethod
    def deliver(self, sender: str, receiver: str, round_idx: int) -> DeliverResult:
        """(전달 여부, 지연 ms)."""


class SharedStreamChannel(Channel):
    """
    연구 계산 재현용: 공유 난수 한 줄을 호출 순서대로 소비한다.

    draw_delay=False 면 손실만 뽑는다 (verify_mrta2.py), True 면 전달된 메시지마다 지연을 하나 더 뽑는다
    (mrta_small.py cbaa_protocol). 호출 순서는 시뮬레이터가 연구 계산과 같게 맞춘다.
    """

    def __init__(self, rng: random.Random, p_loss: float = 0.0, draw_delay: bool = False,
                 delay_ms: Tuple[float, float] = (0.0, 100.0)):
        _check_prob(p_loss)
        self.rng = rng
        self.p_loss = float(p_loss)
        self.draw_delay = bool(draw_delay)
        self.delay_ms = (float(delay_ms[0]), float(delay_ms[1]))

    def deliver(self, sender: str, receiver: str, round_idx: int) -> DeliverResult:
        if self.rng.random() < self.p_loss:
            return False, 0.0
        if self.draw_delay:
            return True, self.rng.uniform(self.delay_ms[0], self.delay_ms[1])
        return True, 0.0


class PerLinkChannel(Channel):
    """
    링크별 파생 난수 + Bernoulli 또는 버스트 손실 + U(lo, hi) ms 지연.

    burst_rho = 0 이면 독립(Bernoulli) 손실 p. 0 < rho < 1 이면 2 상태 Gilbert 모형 — 나쁜 상태에서만
    잃고(손실 1), 정상 상태 확률이 p 이며, 상태의 1 지연 자기상관이 rho 다:
    P(좋음→나쁨) = p·(1−rho), P(나쁨→좋음) = (1−p)·(1−rho). 평균 손실률은 p 로 같고 연속 유실만 길어진다.
    partitions: {(송신자, 수신자): (시작 라운드, 끝 라운드)} — 그 구간에는 그 링크를 끊는다 (끝 포함하지 않음).
    """

    def __init__(self, seed: int, p_loss: float = 0.0,
                 delay_ms: Tuple[float, float] = (0.0, 100.0), burst_rho: float = 0.0,
                 partitions: Optional[Dict[Tuple[str, str], Tuple[int, int]]] = None):
        _check_prob(p_loss)
        if not 0.0 <= burst_rho < 1.0:
            raise ValueError(f'burst_rho 는 [0, 1) 이어야 한다: {burst_rho}')
        lo, hi = float(delay_ms[0]), float(delay_ms[1])
        if lo < 0.0 or hi < lo:
            raise ValueError(f'delay_ms 는 0 ≤ lo ≤ hi 여야 한다: {delay_ms}')
        self.seed = int(seed)
        self.p_loss = float(p_loss)
        self.delay_ms = (lo, hi)
        self.burst_rho = float(burst_rho)
        self.partitions = dict(partitions or {})
        self._rngs: Dict[Tuple[str, str], random.Random] = {}
        self._bad: Dict[Tuple[str, str], bool] = {}

    def _rng(self, link: Tuple[str, str]) -> random.Random:
        rng = self._rngs.get(link)
        if rng is None:
            rng = random.Random(derive_seed(self.seed, link[0], link[1]))
            self._rngs[link] = rng
            if self.burst_rho > 0.0:
                self._bad[link] = rng.random() < self.p_loss   # 정상 분포에서 시작
        return rng

    def deliver(self, sender: str, receiver: str, round_idx: int) -> DeliverResult:
        link = (sender, receiver)
        rng = self._rng(link)
        if self.burst_rho > 0.0:
            bad = self._bad[link]
            flip = (self.p_loss * (1.0 - self.burst_rho) if not bad
                    else (1.0 - self.p_loss) * (1.0 - self.burst_rho))
            if rng.random() < flip:
                bad = not bad
            self._bad[link] = bad
            lost = bad
        else:
            lost = rng.random() < self.p_loss
        delay = rng.uniform(*self.delay_ms)       # 유실이어도 뽑는다 → 링크 난수열이 손실률과 무관하게 정렬
        cut = self.partitions.get(link)
        if cut is not None and cut[0] <= round_idx < cut[1]:
            lost = True
        if lost:
            return False, 0.0
        return True, delay


def _check_prob(p: float) -> None:
    if not 0.0 <= p < 1.0:
        raise ValueError(f'p_loss 는 [0, 1) 이어야 한다: {p}')


__all__ = ['Channel', 'DeliverResult', 'PerLinkChannel', 'SharedStreamChannel', 'derive_seed']
