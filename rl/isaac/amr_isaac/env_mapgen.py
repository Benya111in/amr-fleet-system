"""
지도 변형 평가 env (b1-mapgen) — AmrNavEnv 를 고치지 않고 감싼다. Isaac 앱이 뜬 뒤에만 import 한다.

기본 env(env.py)는 손대지 않는다. 이 하위 클래스는 세 곳만 바꾼다:
  1. 장면 메시 — 타일 0 = 원래 창고 메시(캐시 그대로, 같은 정점·같은 순서), 그 뒤에 변형 지도 타일들의 메시를 세계 좌표의
     다른 자리(map_bundle 의 tile_offset)에 덧붙인다. RayCaster 는 메시 하나만 받으므로 한 메시에 모두 넣는다.
     타일마다 바깥벽이 닫혀 있어 광선이 이웃 타일로 나가지 않는다.
  2. 경로 풀 — 원래 held-out 풀 (번호 그대로) 뒤에 변형 지도에서 다시 계획한 경로(세계 좌표 = 타일 국소 + 이동)를 덧붙인다.
  3. 충돌 — env 마다 자기 타일의 사각형만 본다 (map_tiles.move_and_collide_per_env).
1·3 은 기본 메서드(_setup_scene·_apply_action)를 그대로 부르되, 그 호출 동안만 env.py 모듈의 load_geometry ·
move_and_collide 이름을 바꿔 끼운다 (기본 코드를 복사하지 않는다 — 기본 동작과 어긋날 여지를 줄인다).
정본 safety φ 는 스캔만 입력으로 받으므로 (env_math.CanonicalPhi) 변형 지도의 스캔을 그대로 쓴다. 관측은 자세·경로의
차이만 쓰므로 타일 이동에 불변이다 (tests/test_map_tiles.py).
"""

import contextlib

import numpy as np
import torch

from . import env as E
from . import map_bundle as MB
from . import mapgen as MG
from . import map_tiles as MT
from .contract_torch import F64
from .env_cfg import AmrNavEvalEnvCfg


@contextlib.contextmanager
def _swap(module, name, value):
    old = getattr(module, name)
    setattr(module, name, value)
    try:
        yield
    finally:
        setattr(module, name, old)


class AmrNavMapEnv(E.AmrNavEnv):
    """타일 지도 평가 env (DR 끔, held-out 풀 + 변형 경로)."""

    def __init__(self, cfg: AmrNavEvalEnvCfg, bundle, extra_paths, render_mode=None, **kwargs):
        """
        타일 지도 env — bundle = map_bundle.load_bundle(…) 배열, extra_paths = [(타일, 국소 경로 [n, 2]), …].

        덧붙인 경로 k 의 풀 번호 = 원래 풀 크기 + k (eval_mapgen.py 가 행마다 이 번호를 forced_path_ids 로 준다).
        """
        self._bundle = bundle
        self._offsets = np.asarray(bundle['tile_offset'], dtype=np.float64)
        self._tile_world = []
        self._tile_zr = []
        for t in range(self._offsets.shape[0]):
            obb, zr, _g = MB.tile_rects(bundle, t)
            self._tile_world.append(MG.shift_obb(obb, self._offsets[t]))
            self._tile_zr.append(zr)
        self.mesh_counts = None
        super().__init__(cfg, render_mode, **kwargs)
        nominal_obb = self._geo['obb']
        if not np.array_equal(self._tile_world[0], nominal_obb):
            raise RuntimeError('bundle 의 타일 0 이 캐시 명목 지도와 다르다')
        dev = self.device
        self.tile_rects = MT.padded_tile_rects(self._tile_world, dev)
        if not MT.pad_is_far(np.concatenate(self._tile_world)):
            raise RuntimeError('PAD_RECT 가 타일 지도에 너무 가깝다')
        self.env_tile = torch.zeros(self.num_envs, dtype=torch.long, device=dev)
        self.env_obb = self.tile_rects[self.env_tile]
        self.collide_calls = 0
        self._extend_pool(extra_paths)

    # ------------------------------------------------------------------ 메시
    def _tiled_geometry(self, assets_dir):
        geo = dict(self._orig_load_geometry(assets_dir))
        verts = [np.asarray(geo['verts'], dtype=np.float32)]
        tris = [np.asarray(geo['tris'], dtype=np.int32)]
        n = verts[0].shape[0]
        counts = [int(tris[0].shape[0])]
        for t in range(1, self._offsets.shape[0]):
            if self._bundle['tile_cond'][t] < 0:
                # 명목 지도의 먼 사본: 원래 메시(띠 밖 상자 포함)를 그대로 옮긴다
                v = verts[0].astype(np.float64)
                v[:, 0] += self._offsets[t, 0]
                v[:, 1] += self._offsets[t, 1]
                v, tr = v.astype(np.float32), tris[0]
            else:
                v, tr = MG.box_mesh(self._tile_world[t], self._tile_zr[t])
            verts.append(v)
            tris.append(tr + n)
            n += v.shape[0]
            counts.append(int(tr.shape[0]))
        geo['verts'] = np.concatenate(verts)
        geo['tris'] = np.concatenate(tris)
        self.mesh_counts = counts
        return geo

    def _setup_scene(self):
        self._orig_load_geometry = E.load_geometry
        with _swap(E, 'load_geometry', self._tiled_geometry):
            super()._setup_scene()

    # ------------------------------------------------------------------ 경로 풀
    def _extend_pool(self, extra_paths):
        dev = self.device
        base_pts = self.pool_pts
        base_n = self.pool_n
        add_pts, add_n = [], []
        for t, pts in extra_paths:
            p = np.asarray(pts, dtype=np.float64).copy()
            p[:, 0] += self._offsets[t, 0]
            p[:, 1] += self._offsets[t, 1]
            add_pts.append(p)
            add_n.append(p.shape[0])
        self.n_base_paths = int(self.pool_size)
        if add_pts:
            pts = torch.cat([base_pts, torch.tensor(np.concatenate(add_pts), dtype=F64,
                                                    device=dev)])
            n = torch.cat([base_n, torch.tensor(add_n, dtype=torch.long, device=dev)])
        else:
            pts, n = base_pts, base_n
        offs = torch.cat([torch.zeros(1, dtype=torch.long, device=dev), torch.cumsum(n, 0)])
        if int(n.max()) > self.cfg.path_pad:
            raise ValueError(f'경로 점 {int(n.max())} > path_pad {self.cfg.path_pad}')
        if not torch.equal(offs[:self.n_base_paths + 1], self.pool_offs):
            raise RuntimeError('덧붙인 풀의 앞부분이 원래 풀과 다르다')
        self.pool_pts = pts
        self.pool_offs = offs
        self.pool_n = n
        self.pool_size = int(n.shape[0])
        self.pool_kept = np.arange(self.pool_size)
        # pool_order·pool_len_sorted 는 커리큘럼(학습 전용)만 쓴다 — 평가에서는 쓰지 않으므로 비운다
        self.pool_order = None
        self.pool_len_sorted = None

    # ------------------------------------------------------------------ 충돌
    def set_env_tiles(self, tile_ids):
        """타일 번호 [N] 를 env 마다 정한다 (다음 리셋부터의 충돌 사각형 — eval 라운드마다 부른다)."""
        t = torch.as_tensor(np.asarray(tile_ids), dtype=torch.long, device=self.device)
        if t.shape[0] != self.num_envs:
            raise ValueError(f'타일 번호 {t.shape[0]} ≠ num_envs {self.num_envs}')
        self.env_tile = t
        self.env_obb = self.tile_rects[t]

    def _collide(self, pose, v, w, collided, obb, dt):
        self.collide_calls += 1
        return MT.move_and_collide_per_env(pose, v, w, collided, self.env_obb, dt)

    def _apply_action(self):
        with _swap(E, 'move_and_collide', self._collide):
            super()._apply_action()
