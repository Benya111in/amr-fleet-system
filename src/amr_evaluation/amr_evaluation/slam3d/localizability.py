r"""
단일 스캔 기하 localizability — GT 생성기의 시각체 도형으로 광선을 쏘아 점-대-평면 Fisher 정보를 센다.

연구 산출물 /home/CAPYI_AI/research/artifacts/3d_slam/localizability2.py (보고서 「3D LiDAR SLAM 도입 방안」
1장 표의 출처)를 이 저장소의 정답 기하로 옮긴 것이다. 원본과 같은 가정: 완전 대응, 빔별 거리 잡음 σ, 점마다
법선 n 으로 J = [nᵀ, (p × n)ᵀ] (SE(2) 는 [n_x, n_y, (p × n)_z]), H = JᵀJ / σ². 원본과 다른 점:
  - 기하: 원본은 상자를 명목 자세·축 정렬로 단순화했다 (상자 jitter·표지판·마커·충전소·도크 화물 없음). 여기서는
    gen_warehouse_world.py --gt-dir 의 primitives.json (회전 포함 OBB) 을 그대로 쓴다.
  - S6 통로 끝 막힘 변형을 블로커 상자로 더할 수 있다 (blockers).
절대값이 아니라 순위 비교용이다 (완전 대응 가정의 하한).

    python3 -m amr_evaluation.slam3d.localizability --gt-dir /runs/gt_full --voxel 0 \
        --out /runs/localizability.csv
"""

import argparse
from dataclasses import dataclass
import json
import math
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class Boxes:
    """OBB 묶음: 중심 (N, 3), 회전 (N, 3, 3) (열 = 국소 축의 월드 방향), 반 크기 (N, 3), 라벨 (N,)."""

    c: np.ndarray
    R: np.ndarray
    h: np.ndarray
    label: List[str]

    def __len__(self) -> int:
        """도형 수."""
        return len(self.c)

    def concat(self, other: 'Boxes') -> 'Boxes':
        """두 묶음을 잇는다."""
        return Boxes(np.vstack([self.c, other.c]), np.concatenate([self.R, other.R]),
                     np.vstack([self.h, other.h]), self.label + other.label)


def box(cx: float, cy: float, cz: float, sx: float, sy: float, sz: float, yaw: float = 0.0,
        label: str = 'box') -> Boxes:
    """상자 하나 (yaw 회전)."""
    c, s = math.cos(yaw), math.sin(yaw)
    r = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return Boxes(np.array([[cx, cy, cz]], dtype=float), r[None], np.array([[sx, sy, sz]]) / 2.0,
                 [label])


def boxes_from_primitives(prims: Sequence[dict], slab: float = 0.1) -> Boxes:
    """
    primitives.json 항목 → Boxes. 평면(바닥·천장)은 두께 slab 의 판으로 (법선 반대쪽으로 두께를 준다).

    원기둥·구는 지원하지 않는다 (기본 GT 제외 집합에는 없다 — 있으면 ValueError).
    """
    cs, rs, hs, labels = [], [], [], []
    for p in prims:
        t = np.array(p['T'], dtype=float).reshape(3, 4)
        r, c = t[:, :3], t[:, 3]
        if p['kind'] == 'box':
            h = np.array(p['size'], dtype=float) / 2.0
        elif p['kind'] == 'plane':
            h = np.array([p['size'][0] / 2.0, p['size'][1] / 2.0, slab / 2.0])
            c = c - r[:, 2] * slab / 2.0          # 국소 z = 법선 → 판은 법선 반대쪽
        else:
            raise ValueError(f"{p['name']}: {p['kind']} 는 지원하지 않는다")
        cs.append(c)
        rs.append(r)
        hs.append(h)
        labels.append(p['label'])
    return Boxes(np.array(cs), np.array(rs), np.array(hs), labels)


def load_gt_boxes(gt_dir: str) -> Boxes:
    """gen_warehouse_world.py --gt-dir 출력의 primitives.json → Boxes."""
    with open(f'{gt_dir}/primitives.json', encoding='utf-8') as f:
        return boxes_from_primitives(json.load(f))


def cast(origin: np.ndarray, dirs: np.ndarray, boxes: Boxes,
         chunk: int = 2048) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    광선 (원점 하나, 단위 방향 (M, 3)) 과 OBB 들의 첫 교차 → (거리 t (M,) inf = 없음, 월드 법선 (M, 3), 도형 번호).

    원점이 안에 있는 도형은 맞지 않는 것으로 친다 (센서 자기 몸체·LiDAR 슬롯 — 원본과 같다).
    """
    m = len(dirs)
    t_out = np.full(m, np.inf)
    n_out = np.zeros((m, 3))
    k_out = np.full(m, -1, dtype=np.int64)
    o_l = np.einsum('nij,ni->nj', boxes.R, origin[None, :] - boxes.c)       # (N, 3)
    for s in range(0, m, chunk):
        d = dirs[s:s + chunk]
        d_l = np.einsum('mi,nij->mnj', d, boxes.R)                            # (m, N, 3)
        with np.errstate(divide='ignore', invalid='ignore'):
            inv = 1.0 / d_l
            t1 = (-boxes.h[None] - o_l[None]) * inv
            t2 = (boxes.h[None] - o_l[None]) * inv
        tmin = np.nan_to_num(np.minimum(t1, t2), nan=-np.inf)
        tmax = np.nan_to_num(np.maximum(t1, t2), nan=np.inf)
        tn, tf, ax = tmin.max(2), tmax.min(2), tmin.argmax(2)
        hit = (tn <= tf) & (tf > 0) & (tn > 1e-6)
        tn = np.where(hit, tn, np.inf)
        k = tn.argmin(1)
        rows = np.arange(len(d))
        t = tn[rows, k]
        a = ax[rows, k]
        local_n = np.zeros((len(d), 3))
        local_n[rows, a] = -np.sign(d_l[rows, k, a])
        t_out[s:s + chunk] = t
        n_out[s:s + chunk] = np.einsum('mij,mj->mi', boxes.R[k], local_n)
        k_out[s:s + chunk] = np.where(np.isfinite(t), k, -1)
    return t_out, n_out, k_out


@dataclass
class Sensor:
    """LiDAR 모델: 수직각 [deg], 수평 열 수, 광학 중심 높이 [m], 거리 범위 [m], 앞 숙임 [deg]."""

    name: str
    vertical_deg: Tuple[float, ...]
    n_az: int
    height: float
    rmin: float
    rmax: float
    pitch_deg: float = 0.0

    def rays(self) -> np.ndarray:
        """센서 좌표 단위 방향 (M, 3) — 원본과 같은 순서·각."""
        az = np.linspace(-math.pi, math.pi, self.n_az, endpoint=False)
        v, a = np.meshgrid(np.radians(self.vertical_deg), az, indexing='ij')
        d = np.stack([np.cos(v) * np.cos(a), np.cos(v) * np.sin(a), np.sin(v)], -1).reshape(-1, 3)
        p = math.radians(self.pitch_deg)
        r = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
        return d @ r.T


def _lin(a: float, b: float, n: int) -> Tuple[float, ...]:
    return tuple(float(v) for v in np.linspace(a, b, n))


# 보고서 1장 표의 행 (원본 localizability2.py 의 cfgs 와 같은 값) + 권장 장착(0.90 m, 1800 열)
SENSORS = (
    Sensor('2D 720 @0.20 (현행)', (0.0,), 720, 0.20, 0.10, 25.0),
    Sensor('16ch ±15 @0.45', _lin(-15, 15, 16), 900, 0.45, 0.3, 100.0),
    Sensor('16ch ±15 @0.85', _lin(-15, 15, 16), 900, 0.85, 0.3, 100.0),
    Sensor('16ch ±15 @1.20', _lin(-15, 15, 16), 900, 1.20, 0.3, 100.0),
    Sensor('16ch ±15 @0.85 15° 숙임', _lin(-15, 15, 16), 900, 0.85, 0.3, 100.0, 15.0),
    Sensor('Mid-360형 −7..52 @0.85', _lin(-7, 52, 32), 720, 0.85, 0.1, 40.0),
    Sensor('OS0-64형 ±45 @0.85', _lin(-45, 45, 64), 512, 0.85, 0.3, 50.0),
    Sensor('16ch ±15 @0.90 1800열 (권장안)', _lin(-15, 15, 16), 1800, 0.90, 0.10, 30.0),
)

ROBOT_BODY = (0.6, 0.4, 0.30, 0.03)      # 길이, 폭, 높이, 바닥 틈 [m] (원본과 같은 차체 상자)


def analyse(sensor: Sensor, pos: Tuple[float, float], yaw: float, boxes: Boxes,
            sigma: float = 0.03, voxel: float = 0.0, rcap: float = math.inf,
            along_threshold: float = 0.7,
            ignore_labels: Sequence[str] = ()) -> Dict[str, object]:
    """
    한 자세의 단일 스캔 정보. 진행 방향 = 로봇 x (원본은 통로 축 = 진행 방향으로 둔다).

    ignore_labels 의 도형에 맞은 점은 정보에서 뺀다 (광선은 그대로 가린다). 사전 지도에 없는 블로커(대기 AMR 등)는
    지도 기반 위치추정에서 대응점이 없으므로 이렇게 센다. 스캔 대 스캔 정합(블로커가 정지해 있는 동안)에서는
    블로커도 구속을 주므로 ignore_labels=() 로 따로 센다.

    반환: points, along_points(|n_x| > along_threshold — 진행 축을 구속하는 점), sigma_along_se2 [m],
    sigma_lat_se2 [m], sigma_along_6dof [m], lam_ratio_xy (SE(2) 병진 정보 λmin/λmax), along_sources.
    """
    x0, y0 = pos
    c, s = math.cos(yaw), math.sin(yaw)
    rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    d = sensor.rays() @ rz.T
    o = np.array([x0, y0, sensor.height])
    length, width, height, gap = ROBOT_BODY
    body = box(x0, y0, gap + height / 2, length, width, height, yaw, 'robot')
    allb = boxes.concat(body)
    t, n, k = cast(o, d, allb)
    rmax = min(sensor.rmax, rcap)
    ok = np.isfinite(t) & (t >= sensor.rmin) & (t <= rmax) & (k != len(allb) - 1)
    if ignore_labels:
        lab = np.array(allb.label + [''])
        ok &= ~np.isin(lab[np.where(k >= 0, k, len(allb))], list(ignore_labels))
    p, nn, kk = o + d[ok] * t[ok, None], n[ok], k[ok]
    if voxel and len(p):
        key = np.floor(p / voxel).astype(np.int64)
        _, idx = np.unique(key, axis=0, return_index=True)
        p, nn, kk = p[idx], nn[idx], kk[idx]
    pr, nr = (p - o) @ rz, nn @ rz
    cross = np.cross(pr, nr)
    j6 = np.hstack([nr, cross])
    j3 = np.stack([nr[:, 0], nr[:, 1], cross[:, 2]], 1) if len(nr) else np.zeros((0, 3))
    h6, h3 = j6.T @ j6 / sigma ** 2, j3.T @ j3 / sigma ** 2

    def marg(h: np.ndarray, i: int) -> float:
        try:
            v = float(np.linalg.inv(h)[i, i])
        except np.linalg.LinAlgError:
            return math.inf
        return math.sqrt(v) if v > 0 else math.inf
    w = np.linalg.eigvalsh(h3[:2, :2])
    along = np.abs(nr[:, 0]) > along_threshold
    src: Dict[str, int] = {}
    for i in kk[along]:
        src[allb.label[i]] = src.get(allb.label[i], 0) + 1
    return {'sensor': sensor.name, 'x': x0, 'y': y0, 'yaw': yaw, 'points': int(len(p)),
            'along_points': int(along.sum()), 'sigma_along_se2': marg(h3, 0),
            'sigma_lat_se2': marg(h3, 1), 'sigma_along_6dof': marg(h6, 0),
            'lam_ratio_xy': float(w[0] / w[-1]) if w[-1] > 0 else 0.0,
            'along_sources': dict(sorted(src.items(), key=lambda kv: -kv[1]))}


# ---- S6 통로 끝 막힘 변형 (docs/research/slam3d/evaluation.md §6) ----------------------------------
# 좁은 통로: 중심 (0, -10), y 방향 길이 4 m (y -12 ~ -8), 순폭 0.60 m (x ±0.30). AMR 차체 0.6 x 0.4 x 0.30.
AMR_BODY = (0.6, 0.4, 0.30, 0.03)


def amr(x: float, y: float, yaw: float) -> Boxes:
    """다른 AMR 한 대 (차체 상자만 — 데크 위 화물·마스트 없음)."""
    length, width, height, gap = AMR_BODY
    return box(x, y, gap + height / 2, length, width, height, yaw, 'blocker_amr')


def s6_blockers(variant: str) -> Optional[Boxes]:
    """
    S6 변형별 블로커.

    open    : 막힘 없음 (기준)
    queue   : 통로 토큰을 기다리는 AMR — 북쪽 요청 지점 (0, -5.5) (입구 y = -8 에서 corridor_approach_m 2.5 m),
              통로를 향함 (yaw −π/2). traffic.yaml 의 실제 대기 위치
    queue2  : 양끝 대기 (북 (0, -5.5) + 남 (0, -14.5))
    follow  : 같은 방향 추종 AMR 이 통로 안 1.0 m 뒤 (same_direction: true 로 허용됨) — 대상 (0, -10) 기준 (0, -11.0)
    wall    : 양끝 입구를 0.8 x 0.3 x 0.5 m 화물로 완전히 막음 (최악 — 보고서의 4 m 사거리 상한 대용과 같은 성격)
    """
    if variant == 'open':
        return None
    if variant == 'queue':
        return amr(0.0, -5.5, -math.pi / 2)
    if variant == 'queue2':
        return amr(0.0, -5.5, -math.pi / 2).concat(amr(0.0, -14.5, math.pi / 2))
    if variant == 'follow':
        return amr(0.0, -11.0, math.pi / 2)
    if variant == 'wall':
        return (box(0.0, -7.75, 0.25, 0.8, 0.3, 0.5, 0.0, 'blocker_cargo')
                .concat(box(0.0, -12.25, 0.25, 0.8, 0.3, 0.5, 0.0, 'blocker_cargo')))
    raise ValueError(f'S6 변형 {variant!r} (open | queue | queue2 | follow | wall)')


POSES = {'aisle_centre': ((0.0, -10.0), math.pi / 2),
         'aisle_near_end': ((0.0, -11.5), math.pi / 2),
         'main_aisle': ((3.0, 0.0), 0.0)}


def main(argv: Optional[Sequence[str]] = None) -> int:
    """표 출력 (+ CSV)."""
    ap = argparse.ArgumentParser(description='GT 기하 단일 스캔 localizability (S6 변형 포함)')
    ap.add_argument('--gt-dir', required=True, help='gen_warehouse_world.py --gt-dir 출력')
    ap.add_argument('--poses', nargs='+', default=['aisle_centre'], choices=sorted(POSES))
    ap.add_argument('--variants', nargs='+', default=['open', 'queue', 'queue2', 'follow', 'wall'])
    ap.add_argument('--voxel', type=float, default=0.0, help='점 복셀 솎기 [m] (보고서 표 = 0)')
    ap.add_argument('--rcap', type=float, default=math.inf, help='사거리 상한 [m] (보고서 막힘 대용 = 4)')
    ap.add_argument('--sigma', type=float, default=0.03)
    ap.add_argument('--sensors', nargs='*', type=int, default=None, help='SENSORS 번호 (기본 전부)')
    ap.add_argument('--blockers-as', choices=['outlier', 'landmark'], default='outlier',
                    help='블로커 점: outlier = 지도에 없어 정보에서 뺌 (지도 기반 위치추정, 기본), '
                         'landmark = 구속으로 셈 (정지한 동안의 스캔 대 스캔 정합)')
    ap.add_argument('--out', default='', help='CSV 경로')
    args = ap.parse_args(argv)
    ignore = ('blocker_amr', 'blocker_cargo') if args.blockers_as == 'outlier' else ()
    base = load_gt_boxes(args.gt_dir)
    sensors = [SENSORS[i] for i in args.sensors] if args.sensors else list(SENSORS)
    rows = []
    for pname in args.poses:
        pos, yaw = POSES[pname]
        for var in args.variants:
            extra = s6_blockers(var)
            boxes = base if extra is None else base.concat(extra)
            for sen in sensors:
                r = analyse(sen, pos, yaw, boxes, args.sigma, args.voxel, args.rcap,
                            ignore_labels=ignore)
                r.update(pose=pname, variant=var, voxel=args.voxel, rcap=args.rcap,
                         blockers_as=args.blockers_as)
                rows.append(r)
                print(f"{pname:14s} {var:7s} {sen.name:30s} pts={r['points']:6d} "
                      f"along={r['along_points']:5d} λmin/λmax={r['lam_ratio_xy']:.3g} "
                      f"σ_along={1000 * r['sigma_along_se2']:.3g} mm "
                      f"σ_lat={1000 * r['sigma_lat_se2']:.3g} mm src={r['along_sources']}")
    if args.out:
        import csv
        keys = ['pose', 'variant', 'blockers_as', 'sensor', 'x', 'y', 'yaw', 'voxel', 'rcap',
                'points',
                'along_points', 'lam_ratio_xy', 'sigma_along_se2', 'sigma_lat_se2',
                'sigma_along_6dof', 'along_sources']
        with open(args.out, 'w', encoding='utf-8', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for r in rows:
                w.writerow({k: (json.dumps(r[k], ensure_ascii=False) if k == 'along_sources'
                                else r[k]) for k in keys})
    return 0


if __name__ == '__main__':
    sys.exit(main())
