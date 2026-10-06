"""
C++ 계약 상수(contract.hpp)를 단일 진실 공급원인 파이썬 constants.py 와 대조한다.

golden JSON 에는 상수 블록이 없어서, 헤더의 constexpr 값을 정규식으로 읽어 같은 이름의 파이썬
상수와 1e-12 안에서 비교한다. 헤더 값이 바뀌거나 계약이 바뀌면 이 시험이 깨진다.
"""

import math
import os
import re
import sys

import pytest

PKG = os.environ.get('PKG_DIR', os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CONTRACT = os.environ.get('RL_CONTRACT_DIR',
                          os.path.normpath(os.path.join(PKG, '..', '..', 'rl', 'contract')))
sys.path.insert(0, CONTRACT)

C = pytest.importorskip('amr_rl_contract.constants')

HEADER = os.path.join(PKG, 'include', 'amr_rl_controller', 'core', 'contract.hpp')

# C++ 이름 → 파이썬 이름 (같은 의미)
PAIRS = {
    'kHalfL': 'HALF_L', 'kHalfW': 'HALF_W', 'kVMaxHw': 'V_MAX_HW', 'kVMaxOp': 'V_MAX_OP',
    'kVMin': 'V_MIN', 'kWMax': 'W_MAX', 'kAMax': 'A_MAX', 'kAlphaMax': 'ALPHA_MAX',
    'kJerkMax': 'JERK_MAX', 'kAngJerkMax': 'ANG_JERK_MAX', 'kControllerHz': 'CONTROLLER_HZ',
    'kDt': 'DT', 'kLidarSamples': 'LIDAR_SAMPLES', 'kLidarAngleMin': 'LIDAR_ANGLE_MIN',
    'kLidarAngleMax': 'LIDAR_ANGLE_MAX', 'kLidarRangeMin': 'LIDAR_RANGE_MIN',
    'kLidarRangeMax': 'LIDAR_RANGE_MAX', 'kLidarX': 'LIDAR_X', 'kLidarY': 'LIDAR_Y',
    'kNSectors': 'N_SECTORS', 'kSectorClip': 'SECTOR_CLIP', 'kNPrevActions': 'N_PREV_ACTIONS',
    'kNSubgoals': 'N_SUBGOALS', 'kSubgoalSpacing': 'SUBGOAL_SPACING',
    'kSubgoalDistNorm': 'SUBGOAL_DIST_NORM', 'kGoalClip': 'GOAL_CLIP', 'kKTracks': 'K_TRACKS',
    'kTrackRange': 'TRACK_RANGE', 'kTrackPosNorm': 'TRACK_POS_NORM',
    'kTrackVelNorm': 'TRACK_VEL_NORM', 'kTrackFeats': 'TRACK_FEATS', 'kTtcHorizon': 'TTC_HORIZON',
    'kObstacleRadius': 'OBSTACLE_RADIUS', 'kScanAgeNorm': 'SCAN_AGE_NORM',
    'kTrackAgeNorm': 'TRACK_AGE_NORM', 'kDevPeakNorm': 'DEV_PEAK_NORM',
    'kDevTimerNorm': 'DEV_TIMER_NORM', 'kSimDt': 'SIM_DT', 'kSimTimeMin': 'SIM_TIME_MIN',
    'kSimTimeMax': 'SIM_TIME_MAX', 'kGoalOvershootMargin': 'GOAL_OVERSHOOT_MARGIN',
    'kVoTimeHorizon': 'VO_TIME_HORIZON', 'kVoMargin': 'VO_MARGIN', 'kVoMaxRange': 'VO_MAX_RANGE',
    'kDynSpeedThreshold': 'DYN_SPEED_THRESHOLD', 'kDynFastSpeed': 'DYN_FAST_SPEED',
    'kVoTimeTie': 'VO_TIME_TIE', 'kFpPad': 'FP_PAD',
    'kLatticeTol': 'LATTICE_TOL',
    'kZoneClear': 'ZONE_CLEAR', 'kZoneWarning': 'ZONE_WARNING', 'kZoneCritical': 'ZONE_CRITICAL',
    'kZoneStop': 'ZONE_STOP', 'kGoalXyTol': 'GOAL_XY_TOL', 'kGoalStopSpeed': 'GOAL_STOP_SPEED',
    'kDevOut': 'DEV_OUT', 'kDevBack': 'DEV_BACK',
}
NUM = re.compile(r'constexpr (?:double|int) (k\w+) = ([-+0-9.eE]+);')


def _header_values():
    with open(HEADER, encoding='utf-8') as f:
        return {m.group(1): float(m.group(2)) for m in NUM.finditer(f.read())}


def test_every_pair_is_literal_in_header():
    """대조 대상은 헤더에 리터럴로 적힌 값이어야 한다 (식으로 바뀌면 정규식이 놓친다)."""
    vals = _header_values()
    missing = [k for k in PAIRS if k not in vals]
    assert missing == [], f'헤더에 리터럴이 없는 상수: {missing}'


@pytest.mark.parametrize('cpp,py', sorted(PAIRS.items()))
def test_constant_matches_python(cpp, py):
    """같은 이름의 값이 1e-12 안에서 같다."""
    vals = _header_values()
    assert math.isclose(vals[cpp], float(getattr(C, py)), rel_tol=0.0, abs_tol=1e-12), (cpp, py)


def test_contract_version_and_derived():
    """판 문자열과 파생 상수 (계산식으로 둔 것)."""
    with open(HEADER, encoding='utf-8') as f:
        text = f.read()
    assert f'kContractVersion = "{C.CONTRACT_VERSION}"' in text
    assert f'kContractMajor = {C.CONTRACT_VERSION.split(".")[0]};' in text
    assert math.isclose(math.hypot(0.30, 0.20), C.ROBOT_RADIUS, abs_tol=1e-15)
    assert C.N_CELLS == 11 * 9 and len(C.LATTICE_AV) == 11 and len(C.LATTICE_AW) == 9
    assert math.isclose(C.JERK_STEP_V, 0.2) and math.isclose(C.JERK_STEP_W, 0.6)
