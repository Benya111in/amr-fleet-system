"""영속 저장 시험 — fsync 순서, 손상 파일 → 기억 상실(영구 기권), incarnation."""

import json
import os

import pytest

from amr_fleet.distributed import messages as m
from amr_fleet.distributed import store as store_mod
from amr_fleet.distributed.messages import VoteMsg
from amr_fleet.distributed.ordering import PriorityKey, ReqId, Stamp
from amr_fleet.distributed.store import FileStore, MemoryStore, StoreCorrupt
from amr_fleet.distributed.vote_core import VoteConfig, ZoneAgent

CFG = VoteConfig(('a', 'b', 'c', 'd', 'e'))


def test_file_store_roundtrip_and_missing(tmp_path):
    fs = FileStore(str(tmp_path / 'x' / 'state.json'))
    assert fs.load() is None
    fs.save({'k': [1, 2], 'n': None})
    assert fs.load() == {'k': [1, 2], 'n': None}
    assert sorted(os.listdir(tmp_path / 'x')) == ['state.json']      # 임시 파일이 남지 않는다


@pytest.mark.parametrize('raw', [b'{"trunc', b'\xff\xfe', b'[1, 2]'])
def test_file_store_corrupt(tmp_path, raw):
    p = tmp_path / 'state.json'
    p.write_bytes(raw)
    with pytest.raises(StoreCorrupt):
        FileStore(str(p)).load()


def test_file_store_fsync_order(tmp_path, monkeypatch):
    """쓰기 → fsync(임시 파일) → rename → fsync(디렉터리) 순서여야 한다."""
    calls = []
    fds = {}
    real_open, real_fsync, real_replace, real_write = os.open, os.fsync, os.replace, os.write

    def rec_open(path, flags, mode=0o777):
        fd = real_open(path, flags, mode)
        fds[fd] = 'dir' if os.path.isdir(path) else os.path.basename(path)
        return fd

    def rec_fsync(fd):
        calls.append(('fsync', 'dir' if fds.get(fd) == 'dir' else 'file:' + str(fds.get(fd))))
        return real_fsync(fd)

    def rec_replace(a, b):
        calls.append(('replace', os.path.basename(a), os.path.basename(b)))
        return real_replace(a, b)

    def rec_write(fd, data):
        calls.append(('write', str(fds.get(fd))))
        return real_write(fd, data)

    monkeypatch.setattr(store_mod.os, 'open', rec_open)
    monkeypatch.setattr(store_mod.os, 'fsync', rec_fsync)
    monkeypatch.setattr(store_mod.os, 'replace', rec_replace)
    monkeypatch.setattr(store_mod.os, 'write', rec_write)
    FileStore(str(tmp_path / 'state.json')).save({'a': 1})
    kinds = [c[0] for c in calls]
    i_write = kinds.index('write')
    i_rep = kinds.index('replace')
    tmp_name = calls[i_rep][1]
    assert calls[i_rep][2] == 'state.json' and tmp_name != 'state.json'
    file_syncs = [i for i, c in enumerate(calls) if c == ('fsync', 'file:' + tmp_name)]
    dir_syncs = [i for i, c in enumerate(calls) if c == ('fsync', 'dir')]
    assert file_syncs and i_write < file_syncs[0] < i_rep, calls      # rename 전에 내용 fsync
    assert dir_syncs and dir_syncs[-1] > i_rep, calls                  # rename 뒤 디렉터리 fsync


def test_memory_store_is_json_deep_copy():
    ms = MemoryStore()
    st = {'a': [1]}
    ms.save(st)
    st['a'].append(2)
    assert ms.load() == {'a': [1]}
    ms.corrupt()
    with pytest.raises(StoreCorrupt):
        ms.load()


def test_boot_persists_incarnation_before_any_message(tmp_path):
    fs = FileStore(str(tmp_path / 'a.json'))
    a1, out = ZoneAgent.boot('a', CFG, fs, 0.0)
    assert out == [] and a1.incarnation == 1 and fs.load()['incarnation'] == 1
    a2, _ = ZoneAgent.boot('a', CFG, fs, 1.0)
    assert a2.incarnation == 2 and fs.load()['incarnation'] == 2


def test_votes_survive_restart_with_file_store(tmp_path):
    fs = FileStore(str(tmp_path / 'a.json'))
    a, _ = ZoneAgent.boot('a', CFG, fs, 0.0)
    r = ReqId('b', 1, 1)
    a.handle(VoteMsg(m.REQUEST, 'b', 'a', 0, r, frozenset({'z'}), PriorityKey(0, None, 1),
                     Stamp(10)), 1.0)
    a.handle(VoteMsg(m.ENTER, 'b', 'a', 0, r, frozenset({'z'})), 1.0)
    a2, _ = ZoneAgent.boot('a', CFG, fs, 2.0)
    assert a2.votes['z'].req == r and a2.votes['z'].committed


def test_corrupt_store_makes_voter_amnesic_forever(tmp_path):
    p = tmp_path / 'a.json'
    fs = FileStore(str(p))
    ZoneAgent.boot('a', CFG, fs, 0.0)
    p.write_text('{"version": 1, "incarn')
    a, _ = ZoneAgent.boot('a', CFG, fs, 1.0)
    assert a.amnesic and a.incarnation > 1 << 40
    out = a.handle(VoteMsg(m.REQUEST, 'b', 'a', 0, ReqId('b', 1, 1), frozenset({'z'}),
                           PriorityKey(0, None, 1), Stamp(10)), 1.0)
    assert [(x.kind, x.reason) for x in out] == [(m.NACK, m.R_AMNESIC)]
    assert a.votes == {}
    a2, _ = ZoneAgent.boot('a', CFG, fs, 2.0)            # 다시 기동해도 기권 유지
    assert a2.amnesic
    assert json.loads(p.read_text())['amnesic'] is True


def test_state_version_mismatch_is_treated_as_corrupt():
    ms = MemoryStore({'version': 999})
    a, _ = ZoneAgent.boot('a', CFG, ms, 0.0)
    assert a.amnesic
