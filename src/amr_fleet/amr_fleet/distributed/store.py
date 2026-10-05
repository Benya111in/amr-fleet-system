"""
투표 상태 영속 저장 (프로토콜 명세 2·5 절).

규칙: 투표자·요청자는 상태를 바꾼 뒤 **응답을 돌려주기 전에** save() 를 끝낸다. save() 가 돌아왔으면
그 상태는 크래시를 넘어 살아남아야 한다 (Raft 의 "updated on stable storage before responding").

- FileStore  : 같은 디렉터리의 임시 파일에 쓰고 → flush → fsync(파일) → os.replace → fsync(디렉터리).
               rename 전에 파일을 fsync 하지 않으면 전원 차단 뒤 빈 파일이 남을 수 있고, 디렉터리를
               fsync 하지 않으면 rename 자체가 사라질 수 있다. 순서는 test_dist_store.py 가 고정한다.
- MemoryStore: 시험용. save() 가 곧 영속 (JSON 왕복으로 깊은 복사 — 직렬화 가능성도 같이 검사).

손상된 파일은 StoreCorrupt 를 던진다. 호출자(투표자)는 표를 잊은 것으로 보고 영구 기권해야 한다
(다시 투표하면 이중 투표).
"""

from __future__ import annotations

import json
import os
from typing import Any, Dict, Optional


class StoreCorrupt(Exception):
    """영속 상태를 읽을 수 없음 (손상 · 형식 불일치)."""


class Store:
    """영속 저장 인터페이스."""

    def load(self) -> Optional[Dict[str, Any]]:
        """저장된 상태 (없으면 None). 손상이면 StoreCorrupt."""
        raise NotImplementedError

    def save(self, state: Dict[str, Any]) -> None:
        """상태를 영속화한다. 돌아오면 크래시를 넘어 남아 있어야 한다."""
        raise NotImplementedError


class MemoryStore(Store):
    """시험용 메모리 저장소. 객체가 살아 있는 동안 '디스크' 역할 (에이전트 크래시와 무관)."""

    def __init__(self, initial: Optional[Dict[str, Any]] = None):
        self._blob: Optional[str] = (None if initial is None
                                     else json.dumps(initial, sort_keys=True))
        self.saves = 0

    def load(self) -> Optional[Dict[str, Any]]:
        if self._blob is None:
            return None
        try:
            return json.loads(self._blob)
        except ValueError as e:
            raise StoreCorrupt(str(e)) from e

    def save(self, state: Dict[str, Any]) -> None:
        self._blob = json.dumps(state, sort_keys=True)
        self.saves += 1

    def corrupt(self) -> None:
        """시험용: 저장된 내용을 깨뜨린다."""
        self._blob = '{"truncated'


class FileStore(Store):
    """JSON 파일 + fsync 원자 교체."""

    def __init__(self, path: str):
        self.path = os.path.abspath(path)
        self.dir = os.path.dirname(self.path)
        self.saves = 0

    def load(self) -> Optional[Dict[str, Any]]:
        try:
            with open(self.path, 'rb') as f:
                raw = f.read()
        except FileNotFoundError:
            return None
        try:
            state = json.loads(raw.decode('utf-8'))
        except (UnicodeDecodeError, ValueError) as e:
            raise StoreCorrupt(f'{self.path}: {e}') from e
        if not isinstance(state, dict):
            raise StoreCorrupt(f'{self.path}: 최상위가 객체가 아님')
        return state

    def save(self, state: Dict[str, Any]) -> None:
        data = json.dumps(state, sort_keys=True).encode('utf-8')
        os.makedirs(self.dir, exist_ok=True)
        tmp = f'{self.path}.tmp.{os.getpid()}'
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
        try:
            view = memoryview(data)
            while view:
                n = os.write(fd, view)
                view = view[n:]
            os.fsync(fd)                  # 1) 내용이 디스크에 닿은 뒤에만
        finally:
            os.close(fd)
        os.replace(tmp, self.path)        # 2) 이름을 바꾸고
        dfd = os.open(self.dir, os.O_RDONLY)
        try:
            os.fsync(dfd)                 # 3) 디렉터리 항목(rename)도 영속화
        finally:
            os.close(dfd)
        self.saves += 1
