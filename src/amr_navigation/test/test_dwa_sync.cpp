// DWAController::syncLastCommand 시험 (보너스 1 A2, b1-plugin).
//
// 이 API 는 RL 제어기(amr_rl_controller::RLController)가 내부 DWA 를 폴백으로 부를 때, 그 사이
// 실제로 낸 명령으로 DWA 의 동적 창 중심(직전 명령)을 맞추려고 더했다. 시험 네 묶음:
//  (1) 부르지 않는 경로는 비트 단위로 그대로다: 결정적 시나리오 7 개 × 설정 2 개(코드 기본값·
//      배포 nav2_params.yaml + robot_params.yaml)의 명령 비트를 test_dwa_sync_reference.json 과
//      대조한다. 그 기록은 API 를 더하기 전 코드(a7da8a2)로 만들었다 —
//      DWA_SYNC_WRITE_REFERENCE=<경로> 면 대조 대신 기록하고, DWA_SYNC_REFERENCE_ONLY 로 빌드하면
//      API 를 쓰는 시험을 뺀다 (옛 코드 빌드용).
//      동적 장애물은 넣지 않는다: 트랙 나이를 벽시계로 재므로 실행마다 달라진다.
//  (2) sync(방금 낸 명령) = DWA 가 스스로 낸 것: 두 DWA 의 이후 출력이 비트 단위로 같다
//      (갓 activate 한 DWA 에 sync 해도 같다 — has_last_ 를 켜야 한다).
//  (3) sync(v, w) 뒤 창 중심이 (v, w) 다 — ω 만 어긋난 경우 포함 (1차 물결 변이 R3 생존 보완).
//  (4) 비유한 값 → 창 중심 = 측정 속도 (현재 코어는 NaN 비교가 거짓이라 가드 없이도 같다 —
//      방어용 가드, 등가 변이로 보고). 측정과의 차가 window_reset 밖이면 DWA 규칙대로 측정값.
// 빌드: amr_navigation/CMakeLists.txt 는 gtest 를 이름 목록으로 등록해 이 파일이 저절로 들어가지
// 않는다 → 지금은 amr_rl_controller/CMakeLists.txt 가 이 소스로 test_dwa_sync 를 만든다
// (소유 경로 사정 — amr_navigation 소유자에게 등록 한 줄을 요청했다).
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "amr_navigation/core/grid.hpp"
#include "amr_navigation/dwa_controller.hpp"
#include "amr_navigation/ros_utils.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32.hpp"

using amr_navigation::core::kLethalObstacle;
using amr_navigation::core::OwnedGrid;

namespace
{
class RosEnv : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};
const auto * const g_env = ::testing::AddGlobalTestEnvironment(new RosEnv);

struct Rect
{
  double x0, y0, x1, y1;
};

// 10 x 6 m 방 (test_ros_plugins.cpp 와 같은 구성): racks 면 가운데 1.2 m 통로, extra 는 추가
// 장애물.
// 팽창은 Nav2 식 0.2 / 1.2 / 2.0 을 직접 칠한다 (코스트맵 갱신 스레드 없음).
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> makeCostmap(
  const std::string & name, bool racks, const std::vector<Rect> & extra = {})
{
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", "__node:=" + name});
  opts.parameter_overrides(
    {
      rclcpp::Parameter("global_frame", "map"),
      rclcpp::Parameter("robot_base_frame", "base_footprint"),
      rclcpp::Parameter(
        "footprint", "[[0.30, 0.20], [0.30, -0.20], [-0.30, -0.20], [-0.30, 0.20]]"),
      rclcpp::Parameter("plugins", std::vector<std::string>{"inflation_layer"}),
      rclcpp::Parameter("inflation_layer.plugin", "nav2_costmap_2d::InflationLayer"),
      rclcpp::Parameter("inflation_layer.inflation_radius", 1.2),
      rclcpp::Parameter("inflation_layer.cost_scaling_factor", 2.0),
      rclcpp::Parameter("width", 10),
      rclcpp::Parameter("height", 6),
      rclcpp::Parameter("resolution", 0.05),
    });
  auto cm = std::make_shared<nav2_costmap_2d::Costmap2DROS>(opts);
  cm->on_configure(rclcpp_lifecycle::State());
  OwnedGrid g(200, 120, 0.05);
  if (racks) {
    g.fillRect(0.0, 0.0, 10.0, 0.05, kLethalObstacle);
    g.fillRect(0.0, 5.95, 10.0, 6.0, kLethalObstacle);
    g.fillRect(3.0, 0.0, 7.0, 2.4, kLethalObstacle);
    g.fillRect(3.0, 3.6, 7.0, 6.0, kLethalObstacle);
  }
  for (const auto & r : extra) {
    g.fillRect(r.x0, r.y0, r.x1, r.y1, kLethalObstacle);
  }
  amr_navigation::core::inflate(g, 0.2, 1.2, 2.0);
  auto * costmap = cm->getCostmap();
  for (int y = 0; y < 120; ++y) {
    for (int x = 0; x < 200; ++x) {
      costmap->setCost(x, y, g.at(x, y));
    }
  }
  return cm;
}

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose = amr_navigation::toPoseMsg({x, y, yaw});
  return p;
}

nav_msgs::msg::Path straightPlan(double x0, double y, double len)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const int n = static_cast<int>(std::lround(len / 0.05));
  for (int k = 0; k <= n; ++k) {
    path.poses.push_back(pose(x0 + 0.05 * k, y, 0.0));
  }
  return path;
}

/// 직선 1 m → 반지름 2 m 왼쪽 사분원 → 직선 1 m (0.05 m 간격). 시작 (x0, y0), 방향 +x.
nav_msgs::msg::Path curvedPlan(double x0, double y0)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int k = 0; k <= 20; ++k) {
    path.poses.push_back(pose(x0 + 0.05 * k, y0, 0.0));
  }
  const double r = 2.0;
  const double cx = x0 + 1.0;
  const double cy = y0 + r;
  const int n_arc = static_cast<int>(std::lround(0.5 * M_PI * r / 0.05));
  for (int k = 1; k <= n_arc; ++k) {
    const double a = -0.5 * M_PI + 0.5 * M_PI * k / n_arc;
    path.poses.push_back(pose(cx + r * std::cos(a), cy + r * std::sin(a), a + 0.5 * M_PI));
  }
  for (int k = 1; k <= 20; ++k) {
    path.poses.push_back(pose(cx + r, cy + 0.05 * k, 0.5 * M_PI));
  }
  return path;
}

/// 한 설정 = 하나의 lifecycle 노드 (그 위에 DWA 를 여럿 configure 할 수 있다 — 파라미터 공유).
struct DwaRig
{
  std::string id;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf;
};

/// "defaults": 코드 기본값 + controller_frequency 20. "deploy": 배포 nav2_params.yaml 과
/// robot_params.yaml 을 controller_server 가 읽는 것과 같은 방식(--params-file)으로.
DwaRig makeRig(const std::string & kind)
{
  static int serial = 0;
  DwaRig s;
  s.id = kind;
  rclcpp::NodeOptions opts;
  if (kind == "deploy") {
    // 노드 이름이 controller_server 여야 nav2_params.yaml 의 블록이 붙는다 (robot_params 는 /**).
    // use_sim_time: True 라 /clock 없이는 시각이 0 에 머문다 — 시각에 기대는 갈래(결정 디바운스)가
    // 실행마다 같다.
    opts.arguments(
      {"--ros-args", "-r", "__node:=controller_server", "--params-file", NAV2_PARAMS_FILE,
        "--params-file", ROBOT_PARAMS_FILE});
  } else {
    opts.arguments(
      {"--ros-args", "-r", "__node:=dwa_sync_defaults_" + std::to_string(serial++)});
    opts.parameter_overrides({rclcpp::Parameter("controller_frequency", 20.0)});
  }
  s.node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("dwa_sync_node", opts);
  s.tf = std::make_shared<tf2_ros::Buffer>(s.node->get_clock());
  return s;
}

std::unique_ptr<amr_navigation::DWAController> makeDwa(
  DwaRig & s, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & cm)
{
  auto d = std::make_unique<amr_navigation::DWAController>();
  d->configure(s.node, "DWA", s.tf, cm);
  d->activate();
  return d;
}

std::string hexBits(double x)
{
  uint64_t u = 0;
  std::memcpy(&u, &x, sizeof(u));
  std::ostringstream os;
  os << std::hex << u;
  return os.str();
}

/// 폐루프 (측정 = lag·명령 + (1 − lag)·직전 측정, 운동학 적분 0.05 s). 틱마다 명령 비트를 기록한다.
struct Driver
{
  Driver(amr_navigation::DWAController & d, double x0, double y0, double th0)
  : dwa(d), x(x0), y(y0), th(th0) {}

  amr_navigation::DWAController & dwa;
  double x, y, th;
  double lag{1.0};
  geometry_msgs::msg::Twist vel;
  std::vector<std::vector<std::string>> rec;

  void tick()
  {
    try {
      const auto cmd = dwa.computeVelocityCommands(pose(x, y, th), vel, nullptr);
      const double v = cmd.twist.linear.x;
      const double w = cmd.twist.angular.z;
      rec.push_back({hexBits(v), hexBits(w)});
      vel.linear.x = lag * v + (1.0 - lag) * vel.linear.x;
      vel.angular.z = lag * w + (1.0 - lag) * vel.angular.z;
    } catch (const nav2_core::PlannerException &) {
      rec.push_back({"throw"});
      vel.linear.x = 0.0;
      vel.angular.z = 0.0;
    }
    x += vel.linear.x * std::cos(th) * 0.05;
    y += vel.linear.x * std::sin(th) * 0.05;
    th = std::remainder(th + vel.angular.z * 0.05, 2.0 * M_PI);
  }
  void run(int n)
  {
    for (int k = 0; k < n; ++k) {
      tick();
    }
  }
};

/// 시나리오 7 개 — 경로 추종·곡선·추종 지연·속도 제한·막힘(예외)·목표 접근·경로 밖 출발·적재.
nlohmann::json runScenarios(DwaRig & s)
{
  nlohmann::json out;
  {
    auto cm = makeCostmap("cm_aisle_" + s.id, true);
    auto dwa = makeDwa(s, cm);
    dwa->setPlan(straightPlan(1.0, 3.0, 8.0));
    Driver d{*dwa, 1.0, 3.0, 0.0};
    d.run(120);
    out["aisle_follow"] = d.rec;
  }
  {
    auto cm = makeCostmap("cm_curve_" + s.id, false);
    auto dwa = makeDwa(s, cm);
    dwa->setPlan(curvedPlan(1.0, 1.0));
    Driver d{*dwa, 1.0, 1.0, 0.0};
    d.lag = 0.6;
    d.run(140);
    out["curve_lag"] = d.rec;
  }
  {
    auto cm = makeCostmap("cm_limit_" + s.id, false);
    auto dwa = makeDwa(s, cm);
    dwa->setPlan(straightPlan(0.5, 3.0, 9.0));
    Driver d{*dwa, 0.5, 3.0, 0.0};
    d.run(30);
    dwa->setSpeedLimit(0.3, false);
    d.run(30);
    dwa->setSpeedLimit(50.0, true);
    d.run(30);
    dwa->setSpeedLimit(0.0, false);
    d.run(20);
    out["speed_limit"] = d.rec;
  }
  {
    auto cm = makeCostmap("cm_wall_" + s.id, false, {{2.30, 0.0, 3.0, 6.0}});
    auto dwa = makeDwa(s, cm);
    dwa->setPlan(straightPlan(1.0, 3.0, 5.0));
    Driver d{*dwa, 1.0, 3.0, 0.0};
    d.run(40);
    out["blocked_wall"] = d.rec;
  }
  {
    auto cm = makeCostmap("cm_goal_" + s.id, false);
    auto dwa = makeDwa(s, cm);
    dwa->setPlan(straightPlan(2.0, 3.0, 1.2));
    Driver d{*dwa, 2.0, 3.0, 0.0};
    d.run(60);
    out["goal_approach"] = d.rec;
  }
  {
    auto cm = makeCostmap("cm_offset_" + s.id, false);
    auto dwa = makeDwa(s, cm);
    dwa->setPlan(straightPlan(1.0, 3.0, 7.0));
    Driver d{*dwa, 1.0, 3.5, 0.6};
    d.lag = 0.8;
    d.run(80);
    out["offset_start"] = d.rec;
  }
  {
    auto cm = makeCostmap("cm_payload_" + s.id, false);
    auto dwa = makeDwa(s, cm);
    auto pub_node = std::make_shared<rclcpp::Node>("dwa_sync_mass_pub_" + s.id);
    auto mass_pub = pub_node->create_publisher<std_msgs::msg::Float32>(
      "payload/mass", rclcpp::QoS(1).transient_local());
    std_msgs::msg::Float32 m;
    m.data = 25.0F;
    mass_pub->publish(m);
    for (int k = 0; k < 20; ++k) {
      rclcpp::spin_some(s.node->get_node_base_interface());
      rclcpp::spin_some(pub_node);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    dwa->setPlan(straightPlan(1.0, 3.0, 7.0));
    Driver d{*dwa, 1.0, 3.0, 0.0};
    d.run(40);
    out["payload_25kg"] = d.rec;
  }
  return out;
}
}  // namespace

TEST(DwaSync, OutputsWithoutSyncAreBitIdenticalToPreApiReference)
{
  nlohmann::json got;
  for (const std::string kind : {"defaults", "deploy"}) {
    DwaRig s = makeRig(kind);
    if (kind == "deploy") {
      // 배포 파라미터가 실제로 실렸는지 (nav2_params.yaml DWA.path_horizon 4.0, 코드 기본값 6.0)
      const auto ov = s.node->get_node_parameters_interface()->get_parameter_overrides();
      ASSERT_TRUE(ov.count("DWA.path_horizon"));
      EXPECT_DOUBLE_EQ(ov.at("DWA.path_horizon").get<double>(), 4.0);
      ASSERT_TRUE(ov.count("limits.max_linear_acceleration"));
    }
    got[kind] = runScenarios(s);
  }
  const char * write = std::getenv("DWA_SYNC_WRITE_REFERENCE");
  if (write != nullptr && std::string(write).size() > 0) {
    nlohmann::json j;
    j["description"] =
      "DWAController 출력 비트 (double 16진). syncLastCommand 를 더하기 전 코드로 만든 기준 — "
      "test_dwa_sync.cpp 머리말";
    j["outputs"] = got;
    std::ofstream(write) << j.dump(1) << "\n";
    GTEST_SKIP() << "기준 기록을 썼다: " << write;
  }
  std::ifstream f(DWA_SYNC_REFERENCE_FILE);
  ASSERT_TRUE(f.good()) << DWA_SYNC_REFERENCE_FILE;
  nlohmann::json ref;
  f >> ref;
  const auto & want = ref["outputs"];
  ASSERT_EQ(want.size(), got.size());
  int compared = 0;
  for (const auto & kv : want.items()) {
    ASSERT_TRUE(got.contains(kv.key())) << kv.key();
    for (const auto & sc : kv.value().items()) {
      const auto & a = sc.value();
      const auto & b = got[kv.key()][sc.key()];
      ASSERT_EQ(a.size(), b.size()) << kv.key() << "/" << sc.key();
      for (std::size_t i = 0; i < a.size(); ++i) {
        EXPECT_EQ(a[i], b[i]) << kv.key() << "/" << sc.key() << " tick " << i;
        ++compared;
      }
    }
  }
  EXPECT_EQ(compared, 2 * (120 + 140 + 110 + 40 + 60 + 80 + 40));
}

#ifndef DWA_SYNC_REFERENCE_ONLY

namespace
{
/// 창 중심 (v_c, w_c) 둘레의 1 주기 동적 창 안인가 (배포 한계 a 1.0, α 2.0, Δt 0.05).
void expectInWindow(
  const geometry_msgs::msg::TwistStamped & cmd, double v_c, double w_c, const std::string & what)
{
  constexpr double kDv = 1.0 * 0.05;
  constexpr double kDw = 2.0 * 0.05;
  EXPECT_GE(cmd.twist.linear.x, v_c - kDv - 1e-9) << what;
  EXPECT_LE(cmd.twist.linear.x, v_c + kDv + 1e-9) << what;
  EXPECT_GE(cmd.twist.angular.z, w_c - kDw - 1e-9) << what;
  EXPECT_LE(cmd.twist.angular.z, w_c + kDw + 1e-9) << what;
}

geometry_msgs::msg::Twist twist(double v, double w)
{
  geometry_msgs::msg::Twist t;
  t.linear.x = v;
  t.angular.z = w;
  return t;
}
}  // namespace

// (2) 곡선 경로(ω 가 틱마다 바뀐다)에서 A 는 스스로 계산, B 는 그 틱만 A 의 명령으로 sync →
// 이후 같은 입력에서 둘의 출력이 비트 단위로 같다. C 는 갓 activate 한 DWA (has_last_ 거짓)에
// sync.
TEST(DwaSync, SyncToOwnCommandEqualsSelfEmitted)
{
  DwaRig s = makeRig("deploy");
  auto cm = makeCostmap("cm_eq", false);
  auto a = makeDwa(s, cm);
  auto b = makeDwa(s, cm);
  const auto plan = curvedPlan(1.0, 1.0);
  a->setPlan(plan);
  b->setPlan(plan);
  Driver da{*a, 1.0, 1.0, 0.0};
  da.lag = 0.6;
  // 둘 다 같은 입력으로 40 틱 (곡선 진입)
  for (int k = 0; k < 40; ++k) {
    const auto vb = b->computeVelocityCommands(pose(da.x, da.y, da.th), da.vel, nullptr);
    da.tick();
    ASSERT_EQ(da.rec.back()[0], hexBits(vb.twist.linear.x)) << k;
    ASSERT_EQ(da.rec.back()[1], hexBits(vb.twist.angular.z)) << k;
  }
  // 이 틱: A 만 계산, B 와 C 는 A 의 명령으로 sync
  auto c = makeDwa(s, cm);
  c->setPlan(plan);
  const auto p0 = pose(da.x, da.y, da.th);
  const auto cmd = a->computeVelocityCommands(p0, da.vel, nullptr);
  b->syncLastCommand(cmd.twist.linear.x, cmd.twist.angular.z);
  c->syncLastCommand(cmd.twist.linear.x, cmd.twist.angular.z);
  // C 의 경로 창(PlanWindow 의 hint_)을 A 와 같은 자리로: 같은 자세로 한 번 계산한 뒤 다시 sync
  (void)c->computeVelocityCommands(p0, da.vel, nullptr);
  c->syncLastCommand(cmd.twist.linear.x, cmd.twist.angular.z);
  // 측정·자세는 A 의 명령으로 진행
  da.vel.linear.x = 0.6 * cmd.twist.linear.x + 0.4 * da.vel.linear.x;
  da.vel.angular.z = 0.6 * cmd.twist.angular.z + 0.4 * da.vel.angular.z;
  da.x += da.vel.linear.x * std::cos(da.th) * 0.05;
  da.y += da.vel.linear.x * std::sin(da.th) * 0.05;
  da.th = std::remainder(da.th + da.vel.angular.z * 0.05, 2.0 * M_PI);
  double max_dw = 0.0;
  for (int k = 0; k < 30; ++k) {
    const auto p = pose(da.x, da.y, da.th);
    const auto vb = b->computeVelocityCommands(p, da.vel, nullptr);
    const auto vc = c->computeVelocityCommands(p, da.vel, nullptr);
    da.tick();
    EXPECT_EQ(da.rec.back()[0], hexBits(vb.twist.linear.x)) << "B " << k;
    EXPECT_EQ(da.rec.back()[1], hexBits(vb.twist.angular.z)) << "B " << k;
    EXPECT_EQ(da.rec.back()[0], hexBits(vc.twist.linear.x)) << "C " << k;
    EXPECT_EQ(da.rec.back()[1], hexBits(vc.twist.angular.z)) << "C " << k;
    max_dw = std::max(max_dw, std::abs(vb.twist.angular.z));
  }
  EXPECT_GT(max_dw, 0.2);   // 곡선에서 ω 가 실제로 쓰였다 (ω 를 무시하는 변이가 갈린다)
}

// (3) 창 중심 = sync 한 (v, w). 직선 경로에서 DWA 가 스스로 낸 직전 명령은 (≈0.2, 0).
TEST(DwaSync, SyncSetsWindowCenterIncludingOmega)
{
  DwaRig s = makeRig("deploy");
  auto cm = makeCostmap("cm_win", false);
  auto dwa = makeDwa(s, cm);
  dwa->setPlan(straightPlan(1.0, 3.0, 8.0));
  Driver d{*dwa, 1.0, 3.0, 0.0};
  d.run(4);
  const double v_own = d.vel.linear.x;   // 완전 추종 → 측정 = DWA 의 직전 명령
  ASSERT_NEAR(v_own, 0.2, 1e-6);
  const auto p = pose(d.x, d.y, d.th);
  // v·ω 모두: 중심 (0.5, 0.3), 측정 (0.35, 0) — 차가 window_reset (0.3, 0.5) 안
  dwa->syncLastCommand(0.5, 0.3);
  expectInWindow(dwa->computeVelocityCommands(p, twist(0.35, 0.0), nullptr), 0.5, 0.3, "v·ω");
  // ω 만: 중심 (v_own, 0.3), 측정 (v_own, 0). ω 를 무시하면 ω 창이 0 둘레 [−0.1, 0.1] 이다.
  dwa->syncLastCommand(v_own, 0.3);
  const auto c1 = dwa->computeVelocityCommands(p, twist(v_own, 0.0), nullptr);
  expectInWindow(c1, v_own, 0.3, "ω 만");
  EXPECT_GT(c1.twist.angular.z, 0.15);
  // v 만: 중심 (0.45, 0), 측정 (0.2, 0)
  dwa->syncLastCommand(0.45, 0.0);
  expectInWindow(dwa->computeVelocityCommands(p, twist(0.2, 0.0), nullptr), 0.45, 0.0, "v 만");
}

// (4) 비유한 값과 window_reset 밖.
TEST(DwaSync, NonFiniteOrFarSyncFallsBackToMeasuredCenter)
{
  DwaRig s = makeRig("deploy");
  auto cm = makeCostmap("cm_nan", false);
  auto dwa = makeDwa(s, cm);
  dwa->setPlan(straightPlan(1.0, 3.0, 8.0));
  Driver d{*dwa, 1.0, 3.0, 0.0};
  d.run(4);
  const auto p = pose(d.x, d.y, d.th);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const auto & vw : std::vector<std::pair<double, double>>{
    {nan, 0.0}, {0.3, nan}, {std::numeric_limits<double>::infinity(), 0.0}})
  {
    dwa->syncLastCommand(vw.first, vw.second);
    const auto c = dwa->computeVelocityCommands(p, twist(0.35, 0.2), nullptr);
    EXPECT_TRUE(std::isfinite(c.twist.linear.x) && std::isfinite(c.twist.angular.z));
    expectInWindow(c, 0.35, 0.2, "비유한 → 측정 중심");
  }
  // 측정과 0.55 m/s 차 (> window_reset_v 0.3): sync 해도 DWA 자신의 규칙대로 측정값 중심
  dwa->syncLastCommand(0.9, 0.0);
  expectInWindow(dwa->computeVelocityCommands(p, twist(0.35, 0.0), nullptr), 0.35, 0.0, "먼 sync");
}

#endif  // DWA_SYNC_REFERENCE_ONLY
