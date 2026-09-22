// imu_filter_node: imu/data_raw → 바이어스 보정 + 1차 LPF → imu/data (100 Hz).
//
// 인터페이스 (docs/architecture/components.md §5.2)
//   Sub  imu/data_raw   sensor_msgs/Imu   Gazebo IMU (바이어스 + 백색 잡음, orientation 은 참값)
//   Pub  imu/data       sensor_msgs/Imu   보정·평활 값. orientation_covariance[0] = −1 (방위
//                                         없음): Gazebo orientation 은 노이즈 없는 GT 라 EKF 가
//                                         쓰면 안 된다
//   Srv  imu/calibrate  std_srvs/Trigger  정지 상태에서 calibration_samples 개를 평균해 바이어스
//                                         갱신, logs/calibration/imu_bias_<robot>_<시각>.yaml 과
//                                         imu_bias.csv 기록
// 공분산: 각속도 σ_g² + SE_g², 가속도 σ_a² + SE_a²
// (config/sensors.yaml 잡음 + 바이어스 추정 표준오차).
// LPF 후 분산이 아니라 원시 분산을 넣는다: LPF 출력은 자기상관이 있어, EKF 가 샘플을 독립으로
// 취급하면 분산만 줄인 값은 정보량을 과장한다 (docs/algorithms/kinematics.md §5).

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "amr_localization/imu_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace amr_localization
{

namespace
{
Eigen::Vector3d toVector(const std::vector<double> & v, const char * name)
{
  if (v.size() != 3) {
    throw std::invalid_argument(std::string(name) + " must have 3 elements");
  }
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

std::string vecToYaml(const Eigen::Vector3d & v)
{
  std::ostringstream os;
  os << std::setprecision(9) << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
  return os.str();
}

std::string localTimeString(const char * fmt)
{
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  std::ostringstream os;
  os << std::put_time(&tm, fmt);
  return os.str();
}
}  // namespace

class ImuFilterNode : public rclcpp::Node
{
public:
  using Trigger = std_srvs::srv::Trigger;

  explicit ImuFilterNode(const rclcpp::NodeOptions & options)
  : Node("imu_filter_node", options)
  {
    ImuFilterParams p;
    p.lpf_cutoff_hz = declare_parameter("lpf_cutoff_hz", 20.0);
    p.gyro_bias = toVector(
      declare_parameter<std::vector<double>>("gyro_bias", {0.0, 0.0, 0.0}), "gyro_bias");
    p.accel_bias = toVector(
      declare_parameter<std::vector<double>>("accel_bias", {0.0, 0.0, 0.0}), "accel_bias");
    p.gravity = declare_parameter("gravity", 9.8);
    p.startup_time = declare_parameter("bias_estimation_time", 60.0);
    p.min_startup_samples =
      static_cast<std::size_t>(declare_parameter<std::int64_t>("min_startup_samples", 20));
    p.stationary_gyro_std_max = declare_parameter("stationary_gyro_std_max", 0.01);
    p.stationary_accel_std_max = declare_parameter("stationary_accel_std_max", 0.2);
    p.accel_norm_tolerance = declare_parameter("accel_norm_tolerance", 0.5);
    p.max_gap = declare_parameter("max_gap", 0.5);
    gyro_noise_ = declare_parameter("gyro_noise_stddev", 0.0002);
    accel_noise_ = declare_parameter("accel_noise_stddev", 0.017);
    calibration_samples_ =
      static_cast<std::size_t>(declare_parameter<std::int64_t>("calibration_samples", 1000));
    // 0 이면 샘플 수 / 100 Hz 의 3 배 + 5 s (ROS 시각 기준 → 시뮬레이션이 느려도 오판하지 않는다)
    calibration_timeout_ = declare_parameter("calibration_timeout", 0.0);
    if (calibration_timeout_ <= 0.0) {
      calibration_timeout_ = 3.0 * static_cast<double>(calibration_samples_) / 100.0 + 5.0;
    }
    calibration_dir_ = declare_parameter<std::string>("calibration_dir", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "");

    filter_ = std::make_unique<ImuFilter>(p);

    pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::QoS(10));
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {onImu(*msg);});
    calibrate_srv_ = create_service<Trigger>(
      "imu/calibrate",
      [this](
        std::shared_ptr<rclcpp::Service<Trigger>> service,
        std::shared_ptr<rmw_request_id_t> header,
        std::shared_ptr<Trigger::Request>) {onCalibrate(service, header);});
    timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), [this]() {checkCalibrationTimeout();});

    RCLCPP_INFO(
      get_logger(),
      "imu filter: lpf %.1f Hz, startup bias estimation %.1f s, gyro_bias %s, accel_bias %s",
      p.lpf_cutoff_hz, p.startup_time, vecToYaml(p.gyro_bias).c_str(),
      vecToYaml(p.accel_bias).c_str());
  }

private:
  void onImu(const sensor_msgs::msg::Imu & msg)
  {
    rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() == 0) {
      stamp = now();
    }
    const Eigen::Vector3d gyro(msg.angular_velocity.x, msg.angular_velocity.y,
      msg.angular_velocity.z);
    const Eigen::Vector3d accel(msg.linear_acceleration.x, msg.linear_acceleration.y,
      msg.linear_acceleration.z);
    Eigen::Vector3d gyro_out;
    Eigen::Vector3d accel_out;
    const bool publish = filter_->process(stamp.seconds(), gyro, accel, gyro_out, accel_out);
    handleOutcomes();
    if (!publish) {
      return;
    }

    sensor_msgs::msg::Imu out;
    out.header = msg.header;
    out.header.stamp = stamp;
    if (!frame_id_.empty()) {
      out.header.frame_id = frame_id_;
    }
    out.orientation.w = 1.0;
    out.orientation_covariance.fill(0.0);
    out.orientation_covariance[0] = -1.0;
    out.angular_velocity.x = gyro_out.x();
    out.angular_velocity.y = gyro_out.y();
    out.angular_velocity.z = gyro_out.z();
    out.linear_acceleration.x = accel_out.x();
    out.linear_acceleration.y = accel_out.y();
    out.linear_acceleration.z = accel_out.z();
    out.angular_velocity_covariance.fill(0.0);
    out.linear_acceleration_covariance.fill(0.0);
    const Eigen::Vector3d & gv = filter_->gyroBiasVariance();
    const Eigen::Vector3d & av = filter_->accelBiasVariance();
    for (int i = 0; i < 3; ++i) {
      out.angular_velocity_covariance[i * 4] = gyro_noise_ * gyro_noise_ + gv[i];
      out.linear_acceleration_covariance[i * 4] = accel_noise_ * accel_noise_ + av[i];
    }
    pub_->publish(out);
  }

  void onCalibrate(
    const std::shared_ptr<rclcpp::Service<Trigger>> & service,
    const std::shared_ptr<rmw_request_id_t> & header)
  {
    if (!filter_->calibrating()) {
      filter_->startCalibration(calibration_samples_);
      calibration_started_ = now();
      RCLCPP_INFO(
        get_logger(), "IMU bias calibration started (%zu samples); keep the robot stationary",
        calibration_samples_);
    }
    pending_.emplace_back(service, header);
  }

  void respondAll(bool success, const std::string & message)
  {
    for (auto & [service, header] : pending_) {
      Trigger::Response res;
      res.success = success;
      res.message = message;
      service->send_response(*header, res);
    }
    pending_.clear();
  }

  void checkCalibrationTimeout()
  {
    if (pending_.empty() || !filter_->calibrating()) {
      return;
    }
    const double elapsed = (now() - calibration_started_).seconds();
    if (elapsed > calibration_timeout_) {
      respondAll(false, "calibration timed out: not enough imu/data_raw samples");
      // 진행 중인 수집은 계속되며 끝나면 바이어스가 적용된다 (응답만 실패로 돌려준다)
    }
  }

  void handleOutcomes()
  {
    BiasEstimateOutcome o;
    while (filter_->takeOutcome(o)) {
      const auto & r = o.result;
      if (o.success) {
        RCLCPP_INFO(
          get_logger(), "%s: gyro_bias %s (SE %s) accel_bias %s (SE %s) over %.1f s",
          o.message.c_str(), vecToYaml(r.gyro_bias).c_str(), vecToYaml(r.gyro_se).c_str(),
          vecToYaml(r.accel_bias).c_str(), vecToYaml(r.accel_se).c_str(), o.duration);
      } else {
        RCLCPP_WARN(get_logger(), "%s", o.message.c_str());
      }
      std::string message = o.message;
      if (o.success && !o.startup) {
        const std::string path = writeCalibration(o);
        if (!path.empty()) {
          message += "; written to " + path;
        }
      }
      if (!o.startup) {
        respondAll(o.success, message);
      }
    }
  }

  std::filesystem::path calibrationDirectory() const
  {
    if (!calibration_dir_.empty()) {
      return calibration_dir_;
    }
    const char * ws = std::getenv("ROS_WS");
    return std::filesystem::path(ws ? ws : ".") / "logs" / "calibration";
  }

  std::string writeCalibration(const BiasEstimateOutcome & o)
  {
    const auto & r = o.result;
    try {
      const std::filesystem::path dir = calibrationDirectory();
      std::filesystem::create_directories(dir);
      std::string robot = get_namespace();
      robot.erase(0, robot.find_first_not_of('/'));
      for (auto & c : robot) {
        if (c == '/') {
          c = '_';
        }
      }
      if (robot.empty()) {
        robot = "robot";
      }
      const std::string stamp = localTimeString("%Y%m%d_%H%M%S");
      const auto yaml_path = dir / ("imu_bias_" + robot + "_" + stamp + ".yaml");
      std::ofstream yaml(yaml_path);
      yaml << "# IMU bias calibration (imu_filter_node imu/calibrate, " << stamp << ")\n"
           << "# samples: " << r.samples << ", duration: " << o.duration << " s\n"
           << "# gyro_se: " << vecToYaml(r.gyro_se) << " rad/s, accel_se: "
           << vecToYaml(r.accel_se) << " m/s^2\n"
           << "# gyro_std: " << vecToYaml(r.gyro_std) << " rad/s, accel_std: "
           << vecToYaml(r.accel_std) << " m/s^2\n"
           << "# usage: --params-file <this file> (bias_estimation_time 0 → keep file values)\n"
           << "/**/imu_filter_node:\n"
           << "  ros__parameters:\n"
           << "    gyro_bias: " << vecToYaml(r.gyro_bias) << "\n"
           << "    accel_bias: " << vecToYaml(r.accel_bias) << "\n"
           << "    bias_estimation_time: 0.0\n";
      yaml.close();

      // docs/architecture/sensor_calibration.md §2.4 요약 행
      const auto csv_path = dir / "imu_bias.csv";
      const bool new_file = !std::filesystem::exists(csv_path);
      std::ofstream csv(csv_path, std::ios::app);
      if (new_file) {
        csv << "timestamp,robot,T,N,bax,bay,baz,bgx,bgy,bgz,se_a,se_g\n";
      }
      csv << std::setprecision(9) << stamp << "," << robot << "," << o.duration << ","
          << r.samples << "," << r.accel_bias.x() << "," << r.accel_bias.y() << ","
          << r.accel_bias.z() << "," << r.gyro_bias.x() << "," << r.gyro_bias.y() << ","
          << r.gyro_bias.z() << "," << r.accel_se.maxCoeff() << "," << r.gyro_se.maxCoeff()
          << "\n";
      return yaml_path.string();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "failed to write calibration file: %s", e.what());
      return "";
    }
  }

  std::unique_ptr<ImuFilter> filter_;
  double gyro_noise_{0.0002};
  double accel_noise_{0.017};
  std::size_t calibration_samples_{1000};
  double calibration_timeout_{60.0};
  std::string calibration_dir_;
  std::string frame_id_;
  rclcpp::Time calibration_started_{0, 0, RCL_ROS_TIME};
  std::vector<std::pair<std::shared_ptr<rclcpp::Service<Trigger>>,
    std::shared_ptr<rmw_request_id_t>>> pending_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Service<Trigger>::SharedPtr calibrate_srv_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

}  // namespace amr_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_localization::ImuFilterNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
