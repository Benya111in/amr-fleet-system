// 1차 IIR 저역 통과 필터 (명세 4.2 "IMU 데이터 필터링(저역 통과 필터)").
//
//   y_k = α x_k + (1 − α) y_{k−1},   H(z) = α / (1 − β z⁻¹),  β = 1 − α
//
// α 는 후진 오일러 근사 α = Δt/(τ + Δt) 가 아니라, 이산 주파수 응답이 정확히
// |H(e^{jΩc})|² = 1/2 가 되도록 푼다 (Ωc = 2π f_c Δt):
//   |1 − β e^{−jΩc}|² = 2α²  →  β = (2 − c) − sqrt((2 − c)² − 1),  c = cos Ωc
// 후진 오일러 근사는 f_s = 100 Hz, f_c = 20 Hz 에서 실제 −3 dB 점이 13.7 Hz 로 어긋난다
// (docs/algorithms/kinematics.md §5). 샘플 간격이 바뀌면 매 샘플 α 를 다시 계산한다.

#ifndef AMR_LOCALIZATION__LOW_PASS_FILTER_HPP_
#define AMR_LOCALIZATION__LOW_PASS_FILTER_HPP_

namespace amr_localization
{

/// 스칼라 1차 저역 통과 필터.
class FirstOrderLowPass
{
public:
  /// cutoff_hz ≤ 0 이면 통과(필터 없음).
  explicit FirstOrderLowPass(double cutoff_hz = 0.0);

  /// 차단 주파수 [Hz] 변경. 상태는 유지.
  void setCutoff(double cutoff_hz) {cutoff_hz_ = cutoff_hz;}

  /// 차단 주파수 [Hz].
  double cutoff() const {return cutoff_hz_;}

  /// 정확한 −3 dB 설계의 α. f_c ≤ 0 또는 f_c ≥ Nyquist(1/(2Δt)) 또는 Δt ≤ 0 이면 1(통과).
  static double alphaFor(double cutoff_hz, double dt);

  /// 이산 응답 크기 |H(e^{jΩ})| = α / sqrt(1 − 2β cos Ω + β²), Ω = 2π f Δt [rad/sample].
  static double magnitudeResponse(double alpha, double omega);

  /// 한 샘플 처리. 첫 샘플은 그대로 통과(시동 과도 없음).
  double filter(double x, double dt);

  /// 상태 초기화 (다음 샘플이 첫 샘플).
  void reset() {initialized_ = false;}

  /// 첫 샘플을 받았는지.
  bool initialized() const {return initialized_;}

  /// 마지막 출력.
  double value() const {return y_;}

private:
  double cutoff_hz_;
  bool initialized_{false};
  double y_{0.0};
};

}  // namespace amr_localization

#endif  // AMR_LOCALIZATION__LOW_PASS_FILTER_HPP_
