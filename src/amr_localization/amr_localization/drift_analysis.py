"""
오도메트리 드리프트 분석 (명세 4.2 "누적 오차 특성 분석 / 드리프트 측정 실험", rclpy 비의존).

    ros2 run amr_localization drift_report --input logs/eval/odom_drift/<run>/

입력: odom_drift_experiment 가 쓴 <scenario>_<rep>.csv (COLUMNS). 각 궤적을 시작 자세 기준 상대 궤적
(T_0⁻¹ T_k)으로 바꿔 GT 와 비교한다 — wheel_odom 은 odom 원점, GT 는 월드 원점이라 좌표계가 달라도 된다.
출력: summary.csv (런별 지표), summary.md (시나리오별 평균·표준편차, 폐형 예측, UMBmark).

지표
  final_pos_error  [m]   마지막 샘플의 위치 오차 ‖p_odom − p_gt‖
  final_yaw_error  [rad] 마지막 헤딩 오차
  max_pos_error    [m]
  growth_rate      [m/m] 위치 오차를 누적 주행 거리에 1차 회귀한 기울기 (오차 증가율)
  drift_percent    [%]   final_pos_error / 주행 거리
  yaw_drift        [deg/m] 또는 [deg/rev] (제자리 회전)
  pred_sigma_pos   [m]   wheel_odom 이 발행한 자세 공분산의 sqrt(var_x + var_y) (공분산 전파 검증)
  nees                   (e_x, e_y, e_θ) 의 마할라노비스 제곱 — 공분산이 일관적이면 평균 ≈ 3
체계 오차 (effective_parameters): 직진 런의 거리 배율 s_d = L_odom/L_gt 와 회전 런의 회전 배율
  s_θ = Θ_odom/Θ_gt (부호 있는 누적 회전) 로 유효 바퀴 반지름·간격을 역산한다.
  L_odom = r φ̄, L_gt = r_eff φ̄           → r_eff = r / s_d
  Θ_odom = r Δφ/b, Θ_gt = r_eff Δφ/b_eff  → b_eff = b s_θ r_eff / r = b s_θ / s_d
  (UMBmark 의 E_d/E_b 보정과 같은 목적의 1 차 식별; 좌우 반지름 비 E_d 는 사각 주행 복귀 오차로 본다)
폐형 예측 (docs/algorithms/kinematics.md §4; 곱셈 슬립 σ_s, 주기 Δt, k = σ_s²·|Δs_step|):
  직진 L : σ_θ² = 2kL/b², σ_x² = kL/2, σ_y² ≈ 2kL³/(3b²)
  회전 Θ : σ_θ² = k|Θ|/b   (k 는 바퀴 한 주기 변위 ωbΔt/2 로 계산)
"""

import argparse
import csv
from dataclasses import asdict, dataclass, fields
import math
from pathlib import Path
import sys
from typing import Dict, Iterable, List, Optional, Sequence

import numpy as np

COLUMNS = ['timestamp', 'gt_x', 'gt_y', 'gt_yaw', 'odom_x', 'odom_y', 'odom_yaw',
           'odom_var_x', 'odom_var_y', 'odom_var_yaw', 'odom_cov_xy', 'odom_cov_xyaw',
           'odom_cov_yyaw', 'ekf_x', 'ekf_y', 'ekf_yaw']


def wrap(a):
    """각을 (−π, π] 로 (배열 가능)."""
    return np.arctan2(np.sin(a), np.cos(a))


def relative_track(x: np.ndarray, y: np.ndarray, yaw: np.ndarray):
    """첫 자세 기준 상대 궤적 T_0⁻¹ T_k."""
    c, s = math.cos(yaw[0]), math.sin(yaw[0])
    dx, dy = x - x[0], y - y[0]
    return c * dx + s * dy, -s * dx + c * dy, wrap(yaw - yaw[0])


def relative_covariance(cov: np.ndarray, yaw0: float) -> np.ndarray:
    """자세 공분산(odom 프레임)을 시작 헤딩 기준 프레임으로 회전한다 (R Σ Rᵀ)."""
    c, s = math.cos(yaw0), math.sin(yaw0)
    r = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    return r @ cov @ r.T


def path_length(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """누적 주행 거리 [m]."""
    return np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])


def total_rotation(yaw: np.ndarray) -> float:
    """누적 회전량 |Σ Δθ| 가 아닌 Σ |Δθ| [rad]."""
    return float(np.sum(np.abs(wrap(np.diff(yaw))))) if yaw.size > 1 else 0.0


def net_rotation(yaw: np.ndarray) -> float:
    """부호 있는 누적 회전 Σ wrap(Δθ) [rad] (감김을 풀어 2π 이상도 표현)."""
    return float(np.sum(wrap(np.diff(yaw)))) if yaw.size > 1 else 0.0


def predicted_straight(length: float, sigma_s: float, separation: float,
                       ref_distance: float = 0.01) -> Dict[str, float]:
    """
    직진 L 의 폐형 표준편차 (x, y, θ).

    거리당 슬립 Var(Δs_i) = k|Δs_i|, k = σ_s² ℓ_ref (속도·주기와 무관, kinematics.md §4.1).
    """
    k = sigma_s ** 2 * ref_distance
    return {'sigma_x': math.sqrt(k * length / 2.0),
            'sigma_y': math.sqrt(2.0 * k * length ** 3 / (3.0 * separation ** 2)),
            'sigma_yaw': math.sqrt(2.0 * k * length / separation ** 2)}


def predicted_rotation(angle: float, sigma_s: float, separation: float,
                       ref_distance: float = 0.01) -> Dict[str, float]:
    """제자리 회전 Θ 의 폐형 헤딩 표준편차 (바퀴마다 |Θ|b/2 굴림 → Var θ = k|Θ|/b)."""
    k = sigma_s ** 2 * ref_distance
    return {'sigma_yaw': math.sqrt(k * abs(angle) / separation)}


@dataclass
class RunMetrics:
    """런 1 회 지표."""

    scenario: str
    run: str
    samples: int
    duration: float
    path_length: float
    rotation: float
    odom_path_length: float
    net_rotation: float
    odom_net_rotation: float
    final_pos_error: float
    final_x_error: float
    final_y_error: float
    final_yaw_error: float
    max_pos_error: float
    growth_rate: float
    drift_percent: float
    yaw_drift_deg_per_m: float
    yaw_drift_deg_per_rev: float
    pred_sigma_pos: float
    pred_sigma_yaw: float
    nees: float
    ekf_final_pos_error: float
    ekf_final_yaw_error: float


def load_csv(path: Path) -> Dict[str, np.ndarray]:
    """CSV → 열 이름별 배열 (빈 칸은 NaN)."""
    with path.open(newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    out = {}
    for col in COLUMNS:
        out[col] = np.array([float(r[col]) if r.get(col) not in (None, '') else math.nan
                             for r in rows], dtype=float)
    return out


def analyze_run(data: Dict[str, np.ndarray], scenario: str, run: str) -> RunMetrics:
    """한 런의 지표. 샘플이 2 개 미만이면 ValueError."""
    n = data['timestamp'].size
    if n < 2:
        raise ValueError(f'{scenario}/{run}: samples {n} < 2')
    gx, gy, gth = relative_track(data['gt_x'], data['gt_y'], data['gt_yaw'])
    ox, oy, oth = relative_track(data['odom_x'], data['odom_y'], data['odom_yaw'])
    ex, ey, eth = ox - gx, oy - gy, wrap(oth - gth)
    err = np.hypot(ex, ey)
    s = path_length(gx, gy)
    length = float(s[-1])
    rotation = total_rotation(gth)
    growth = float(np.polyfit(s, err, 1)[0]) if length > 0.5 else math.nan

    # 공분산: 마지막 샘플 − 첫 샘플 (wheel_odom/reset 직후면 첫 샘플 ≈ 0), 시작 헤딩 프레임으로 회전
    def cov_at(i: int) -> np.ndarray:
        return np.array([
            [data['odom_var_x'][i], data['odom_cov_xy'][i], data['odom_cov_xyaw'][i]],
            [data['odom_cov_xy'][i], data['odom_var_y'][i], data['odom_cov_yyaw'][i]],
            [data['odom_cov_xyaw'][i], data['odom_cov_yyaw'][i], data['odom_var_yaw'][i]]])
    cov = relative_covariance(cov_at(-1) - cov_at(0), float(data['odom_yaw'][0]))
    e = np.array([ex[-1], ey[-1], eth[-1]])
    nees = math.nan
    if np.all(np.isfinite(cov)) and np.linalg.det(cov) > 0.0:
        nees = float(e @ np.linalg.solve(cov, e))

    ekf_pos = ekf_yaw = math.nan
    if np.all(np.isfinite(data['ekf_x'])):
        kx, ky, kth = relative_track(data['ekf_x'], data['ekf_y'], data['ekf_yaw'])
        ekf_pos = float(math.hypot(kx[-1] - gx[-1], ky[-1] - gy[-1]))
        ekf_yaw = float(wrap(kth[-1] - gth[-1]))

    return RunMetrics(
        scenario=scenario, run=run, samples=n,
        duration=float(data['timestamp'][-1] - data['timestamp'][0]),
        path_length=length, rotation=rotation,
        odom_path_length=float(path_length(ox, oy)[-1]),
        net_rotation=net_rotation(gth), odom_net_rotation=net_rotation(oth),
        final_pos_error=float(err[-1]), final_x_error=float(ex[-1]),
        final_y_error=float(ey[-1]), final_yaw_error=float(eth[-1]),
        max_pos_error=float(err.max()), growth_rate=growth,
        drift_percent=100.0 * float(err[-1]) / length if length > 0.5 else math.nan,
        yaw_drift_deg_per_m=(math.degrees(abs(eth[-1])) / length if length > 0.5
                             else math.nan),
        yaw_drift_deg_per_rev=(math.degrees(abs(eth[-1])) / (rotation / (2 * math.pi))
                               if rotation > 0.5 else math.nan),
        pred_sigma_pos=float(math.sqrt(max(cov[0, 0] + cov[1, 1], 0.0))),
        pred_sigma_yaw=float(math.sqrt(max(cov[2, 2], 0.0))),
        nees=nees, ekf_final_pos_error=ekf_pos, ekf_final_yaw_error=ekf_yaw)


def umbmark(runs: Sequence[RunMetrics]) -> Optional[Dict[str, float]]:
    """
    시계/반시계 사각 주행 복귀 오차로 체계 오차를 구한다 (UMBmark, Borenstein & Feng 1996).

    E_max,syst = max(|c_cw|, |c_ccw|), c = 복귀 오차 (x, y) 평균.
    """
    cw = [(r.final_x_error, r.final_y_error) for r in runs if r.scenario == 'square_cw']
    ccw = [(r.final_x_error, r.final_y_error) for r in runs if r.scenario == 'square_ccw']
    if not cw or not ccw:
        return None
    c_cw = np.mean(np.array(cw), axis=0)
    c_ccw = np.mean(np.array(ccw), axis=0)
    return {'c_cw_x': float(c_cw[0]), 'c_cw_y': float(c_cw[1]),
            'c_ccw_x': float(c_ccw[0]), 'c_ccw_y': float(c_ccw[1]),
            'e_max_syst': float(max(np.hypot(*c_cw), np.hypot(*c_ccw)))}


def effective_parameters(runs: Sequence[RunMetrics], wheel_radius: float,
                         separation: float) -> Optional[Dict[str, float]]:
    """
    직진 런의 거리 배율과 회전 런의 회전 배율로 유효 r, b 를 역산 (모듈 머리 주석의 식).

    직진 런(주행 > 0.5 m)이나 회전 런(회전 > 0.5 rad)이 없으면 None.
    """
    s_d = [r.odom_path_length / r.path_length for r in runs
           if r.scenario == 'straight' and r.path_length > 0.5]
    s_th = [r.odom_net_rotation / r.net_rotation for r in runs
            if r.scenario == 'rotate' and abs(r.net_rotation) > 0.5]
    if not s_d or not s_th:
        return None
    d, th = float(np.mean(s_d)), float(np.mean(s_th))
    return {'distance_scale': d, 'heading_scale': th,
            'wheel_radius_eff': wheel_radius / d, 'separation_eff': separation * th / d}


def _stat(values: Iterable[float]) -> str:
    v = np.array([x for x in values if math.isfinite(x)])
    if v.size == 0:
        return 'n/a'
    if v.size == 1:
        return f'{v[0]:.4f}'
    return f'{v.mean():.4f} ± {v.std(ddof=1):.4f}'


def summarize(runs: Sequence[RunMetrics], meta: Optional[Dict[str, float]] = None) -> str:
    """시나리오별 Markdown 요약."""
    lines = ['# 오도메트리 드리프트 실험 요약', '']
    if meta:
        lines.append('조건: ' + ', '.join(f'{k}={v}' for k, v in meta.items()))
        lines.append('')
    lines += ['| 시나리오 | 런 | 주행 [m] | 회전 [rad] | 최종 위치 오차 [m] | 최대 [m] | '
              '최종 헤딩 오차 [deg] | 증가율 [m/m] | 공분산 σ_pos [m] | NEES | EKF 최종 [m] |',
              '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for scenario in sorted({r.scenario for r in runs}):
        rs = [r for r in runs if r.scenario == scenario]
        lines.append(
            f'| {scenario} | {len(rs)} | {_stat(r.path_length for r in rs)} | '
            f'{_stat(r.rotation for r in rs)} | {_stat(r.final_pos_error for r in rs)} | '
            f'{_stat(r.max_pos_error for r in rs)} | '
            f'{_stat(math.degrees(abs(r.final_yaw_error)) for r in rs)} | '
            f'{_stat(r.growth_rate for r in rs)} | {_stat(r.pred_sigma_pos for r in rs)} | '
            f'{_stat(r.nees for r in rs)} | {_stat(r.ekf_final_pos_error for r in rs)} |')
    u = umbmark(runs)
    if u:
        lines += ['', f"UMBmark: c_cw = ({u['c_cw_x']:.4f}, {u['c_cw_y']:.4f}) m, "
                      f"c_ccw = ({u['c_ccw_x']:.4f}, {u['c_ccw_y']:.4f}) m, "
                      f"E_max,syst = {u['e_max_syst']:.4f} m"]
    if meta and 'wheel_radius' in meta and 'separation' in meta:
        e = effective_parameters(runs, meta['wheel_radius'], meta['separation'])
        if e:
            lines += ['', f"체계 오차 (유효 파라미터): 거리 배율 L_odom/L_gt = "
                          f"{e['distance_scale']:.5f}, 회전 배율 Θ_odom/Θ_gt = "
                          f"{e['heading_scale']:.5f} → r_eff = {e['wheel_radius_eff']:.5f} m "
                          f"(공칭 {meta['wheel_radius']}), b_eff = {e['separation_eff']:.5f} m "
                          f"(공칭 {meta['separation']})"]
    if meta and all(k in meta for k in ('sigma_s', 'separation')):
        ref = meta.get('slip_reference_distance', 0.01)
        lines += ['', f'## 폐형 예측 (거리당 슬립 잡음만 σ_s {meta["sigma_s"]}, ℓ_ref {ref} m, '
                      '체계 오차 0)', '']
        for r in runs:
            if r.scenario == 'straight':
                p = predicted_straight(r.path_length, meta['sigma_s'], meta['separation'], ref)
                lines.append(f"- straight {r.run}: L={r.path_length:.2f} m → σ_x "
                             f"{p['sigma_x']:.4f} m, σ_y {p['sigma_y']:.4f} m, σ_θ "
                             f"{math.degrees(p['sigma_yaw']):.3f} deg")
                break
        for r in runs:
            if r.scenario == 'rotate':
                p = predicted_rotation(r.rotation, meta['sigma_s'], meta['separation'], ref)
                lines.append(f"- rotate {r.run}: Θ={r.rotation:.2f} rad → σ_θ "
                             f"{math.degrees(p['sigma_yaw']):.3f} deg")
                break
    return '\n'.join(lines) + '\n'


def write_metrics_csv(runs: Sequence[RunMetrics], path: Path) -> None:
    """런별 지표 CSV."""
    names = [f.name for f in fields(RunMetrics)]
    with path.open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=names)
        w.writeheader()
        for r in runs:
            w.writerow(asdict(r))


def analyze_directory(directory: Path, meta: Optional[Dict[str, float]] = None
                      ) -> List[RunMetrics]:
    """<scenario>_<rep>.csv 전부 분석 → summary.csv, summary.md."""
    runs = []
    for path in sorted(directory.glob('*.csv')):
        if path.name.startswith('summary'):
            continue
        scenario, _, rep = path.stem.rpartition('_')
        if not scenario:
            continue
        runs.append(analyze_run(load_csv(path), scenario, rep))
    if runs:
        write_metrics_csv(runs, directory / 'summary.csv')
        (directory / 'summary.md').write_text(summarize(runs, meta), encoding='utf-8')
    return runs


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI 진입점."""
    ap = argparse.ArgumentParser(description='wheel odometry drift report')
    ap.add_argument('--input', required=True, help='odom_drift_experiment 출력 디렉토리')
    ap.add_argument('--sigma-s', type=float, default=0.01, help='슬립 잡음 σ_s (폐형 예측)')
    ap.add_argument('--separation', type=float, default=0.36, help='바퀴 간격 b [m]')
    ap.add_argument('--wheel-radius', type=float, default=0.0825, help='바퀴 반지름 r [m]')
    ap.add_argument('--slip-reference-distance', type=float, default=0.01,
                    help='슬립 기준 굴림 거리 ℓ_ref [m] (wheel_odometry.yaml)')
    args = ap.parse_args(argv)
    meta = {'sigma_s': args.sigma_s, 'separation': args.separation,
            'wheel_radius': args.wheel_radius,
            'slip_reference_distance': args.slip_reference_distance}
    runs = analyze_directory(Path(args.input), meta)
    if not runs:
        print(f'no runs in {args.input}', file=sys.stderr)
        return 1
    print((Path(args.input) / 'summary.md').read_text(encoding='utf-8'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
