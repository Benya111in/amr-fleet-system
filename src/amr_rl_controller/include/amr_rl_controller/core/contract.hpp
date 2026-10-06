// RL 내비게이션 계약 v1.2.0 상수 — rl/contract/amr_rl_contract/constants.py 의 C++ 사본.
// v1.2.0 은 보상만 바꿨다 (관측·행동·마스크·ONNX 입출력은 v1.1.0 과 같다, contract.md 변경 이력) →
// 이 플러그인의 계산은 그대로이고 판 문자열만 올린다. 모델은 주 버전이 같으면 받는다 (onnx_io.md).
//
// 단일 진실 공급원은 파이썬 constants.py 다. 이 파일의 값은 golden JSON 의 "constants" 블록과
// test/test_contract_golden.cpp 가 대조한다 (값이 어긋나면 시험이 깨진다).
// 출처 표기: R = config/robot_params.yaml, S = config/sensors.yaml,
// N = src/amr_navigation/config/nav2_params.yaml, V = velocity_profiler.yaml.
#ifndef AMR_RL_CONTROLLER__CORE__CONTRACT_HPP_
#define AMR_RL_CONTROLLER__CORE__CONTRACT_HPP_

#include <cmath>

namespace amr_rl_controller
{
namespace contract
{

constexpr const char * kContractVersion = "1.2.0";
constexpr int kContractMajor = 1;

// 로봇 기하·한계
constexpr double kHalfL = 0.30;            // R:15 (0.60 / 2)
constexpr double kHalfW = 0.20;            // R:16 (0.40 / 2)
inline double robotRadius() {return std::hypot(kHalfL, kHalfW);}   // 0.3606 (외접원)
constexpr double kVMaxHw = 2.0;            // R:57
constexpr double kVMaxOp = 1.0;            // N:104 (운용 상한 v_cap 기본값)
constexpr double kVMin = 0.0;              // N:105
constexpr double kWMax = 1.5;              // R:60
constexpr double kAMax = 1.0;              // R:59
constexpr double kAlphaMax = 2.0;          // R:61
constexpr double kJerkMax = 2.0;           // R:62
constexpr double kAngJerkMax = 6.0;        // V:11

// 주기
constexpr double kControllerHz = 20.0;     // N:75
constexpr double kDt = 0.1;                // 정책 결정 주기 (LiDAR 10 Hz, contract.md §3)

// LiDAR
constexpr int kLidarSamples = 720;                 // S:28
constexpr double kLidarAngleMin = -3.14159265;     // S:38
constexpr double kLidarAngleMax = 3.13286601;      // S:39
constexpr double kLidarAngleInc = (kLidarAngleMax - kLidarAngleMin) / (kLidarSamples - 1);
constexpr double kLidarRangeMin = 0.10;            // S:40
constexpr double kLidarRangeMax = 25.0;            // S:41
constexpr double kLidarX = 0.15;                   // S:44
constexpr double kLidarY = 0.0;

// 관측
constexpr int kNSectors = 180;
constexpr int kBeamsPerSector = kLidarSamples / kNSectors;   // 4
constexpr double kSectorClip = 10.0;
constexpr int kNPrevActions = 3;
constexpr int kNSubgoals = 5;
constexpr double kSubgoalSpacing = 0.5;
constexpr double kSubgoalDistNorm = 2.5;
constexpr double kGoalClip = 10.0;
constexpr int kKTracks = 8;
constexpr double kTrackRange = 10.0;
constexpr double kTrackPosNorm = 10.0;
constexpr double kTrackVelNorm = 2.0;
constexpr int kTrackFeats = 8;
constexpr double kTtcHorizon = 5.0;
constexpr double kObstacleRadius = 0.30;   // N:341
constexpr double kScanAgeNorm = 0.3;
constexpr double kTrackAgeNorm = 0.5;
constexpr double kDevPeakNorm = 1.0;
constexpr double kDevTimerNorm = 5.0;

// 관측 블록 (OBS_LAYOUT 순서)
constexpr int kObsLidar = 0;
constexpr int kObsEgo = kObsLidar + kNSectors;                  // 180
constexpr int kObsTiming = kObsEgo + 6 + 2 * kNPrevActions;     // 192
constexpr int kObsPath = kObsTiming + 3;                        // 195
constexpr int kObsGoal = kObsPath + 3 * kNSubgoals + 3;         // 213
constexpr int kObsSafety = kObsGoal + 4;                        // 217
constexpr int kObsDev = kObsSafety + 7;                         // 224
constexpr int kObsTracks = kObsDev + 3;                         // 227
constexpr int kObsDim = kObsTracks + kKTracks * kTrackFeats;    // 291

// DWA 판정 상수 (비교군 C 마스크)
constexpr double kSimDt = 0.1;             // N:136
constexpr double kSimTimeMin = 1.5;        // N:137
constexpr double kSimTimeMax = 2.5;        // N:138
constexpr double kGoalOvershootMargin = 0.1;   // N:178
constexpr double kVoTimeHorizon = 2.0;     // N:185
constexpr double kVoMargin = 0.3;          // N:186
constexpr double kVoMaxRange = 6.0;        // N:187
constexpr double kDynSpeedThreshold = 0.2;     // N:188
constexpr double kDynFastSpeed = 0.5;      // N:189
constexpr double kVoTimeTie = 0.02;        // N:191
constexpr double kFpPad = 0.0;

// 비교군 C 격자 (contract.md §5.2)
constexpr int kNAv = 11;                   // a_v ∈ {−1.0, −0.8, …, 1.0}
constexpr int kNAw = 9;                    // a_ω ∈ {−2.0, −1.5, …, 2.0}
constexpr int kNCells = kNAv * kNAw;       // 99
constexpr int kFallbackIndex = kNCells;    // 99
constexpr int kNActionsC = kNCells + 1;    // 100
constexpr double kJerkStepV = kJerkMax * kDt;       // 0.2
constexpr double kJerkStepW = kAngJerkMax * kDt;    // 0.6
constexpr double kLatticeTol = 1e-6;
inline double latticeAv(int iv) {return std::round((-1.0 + 0.2 * iv) * 1e10) / 1e10;}
inline double latticeAw(int iw) {return std::round((-2.0 + 0.5 * iw) * 1e10) / 1e10;}

static_assert(kObsDim == 291, "OBS_DIM (observation.OBS_LAYOUT)");
static_assert(kNCells == 99 && kFallbackIndex == 99 && kNActionsC == 100, "N_ACTIONS_C");

// 안전 존
constexpr int kZoneClear = 0;
constexpr int kZoneWarning = 1;
constexpr int kZoneCritical = 2;
constexpr int kZoneStop = 3;

// 목표·이탈
constexpr double kGoalXyTol = 0.10;        // N:98
constexpr double kGoalStopSpeed = 0.05;
constexpr double kDevOut = 0.30;
constexpr double kDevBack = 0.15;

}  // namespace contract
}  // namespace amr_rl_controller

#endif  // AMR_RL_CONTROLLER__CORE__CONTRACT_HPP_
