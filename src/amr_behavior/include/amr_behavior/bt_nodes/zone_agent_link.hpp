// 분산 플릿 에이전트 연결 (보너스 4 P3): fleet_agent/state 구독 + zone_command 클라이언트.
//
// 허가의 근거는 에이전트 상태의 held 하나뿐이다 (허가 목록, fail-closed).
// 상태가 max_age 보다 오래되면 (에이전트가 죽었거나 끊김) 아무것도 허가되지 않은 것으로 본다.
// 미래 stamp 도 future_tolerance 를 넘으면 신선하지 않다 — sim 리셋 · 시계 혼용(벽시계 stamp
// 를 sim 시계로 읽음)이면 age 가 크게 음수라 "언제나 신선" 이 된다 (fail-open, P3 리뷰 §8-13).
// 프로세스마다 /clock 을 받는 시각이 조금 달라 작은 음수는 정상이므로 허용치를 둔다 (기본 0.5 s).
// 모든 존 BT 노드가 이 연결 하나를 나눠 쓴다 (registerZoneNodes 가 만든다). 시각은 모두 노드 시계
// (use_sim_time 이면 /clock) — 에이전트 · 상태 stamp 와 같은 축이다 (§8-14).
#ifndef AMR_BEHAVIOR__BT_NODES__ZONE_AGENT_LINK_HPP_
#define AMR_BEHAVIOR__BT_NODES__ZONE_AGENT_LINK_HPP_

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "amr_behavior/bt_nodes/ros_node_params.hpp"
#include "amr_msgs/msg/agent_state.hpp"
#include "amr_msgs/srv/zone_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_behavior
{

/// "a;b" · "zone:a,zone:b" → {"zone:a", "zone:b"} (접두사가 없으면 prefix 를 붙인다).
inline std::vector<std::string> parseResources(
  const std::string & text, const std::string & prefix = "zone:")
{
  std::vector<std::string> out;
  std::string item;
  std::stringstream ss(text);
  while (std::getline(ss, item, ';')) {
    std::stringstream inner(item);
    std::string part;
    while (std::getline(inner, part, ',')) {
      part.erase(0, part.find_first_not_of(" \t"));
      part.erase(part.find_last_not_of(" \t") + 1);
      if (part.empty()) {
        continue;
      }
      if (part.find(':') == std::string::npos) {
        part = prefix + part;
      }
      if (std::find(out.begin(), out.end(), part) == out.end()) {
        out.push_back(part);
      }
    }
  }
  return out;
}

/// 노드 시계(sim) 기준 경과 시간 — 존 노드의 시간 초과용 (P3 리뷰 §8-14: 벽시계 시간 초과는
/// RTF 0.25 에서 sim 기준 4 배 빨리 끝나 투표 재시도 · 백오프(sim)보다 먼저 포기한다).
/// /clock 을 받기 전(시각 0)에는 재지 않고 첫 0 아닌 시각부터 잰다. 시계가 뒤로 가면(sim 리셋)
/// 그 시각부터 다시 잰다.
class SimStopwatch
{
public:
  void start(const rclcpp::Time & now)
  {
    started_ = now.nanoseconds() != 0;
    if (started_) {
      start_ = now;
    }
  }

  /// 시작 뒤 경과 [s] (시작 전 · 시각 0 이면 0).
  double elapsedS(const rclcpp::Time & now)
  {
    if (now.nanoseconds() == 0) {
      return 0.0;
    }
    if (!started_ || now.get_clock_type() != start_.get_clock_type() || now < start_) {
      start_ = now;
      started_ = true;
    }
    return (now - start_).seconds();
  }

private:
  rclcpp::Time start_;
  bool started_{false};
};

class ZoneAgentLink
{
public:
  using State = amr_msgs::msg::AgentState;
  using Command = amr_msgs::srv::ZoneCommand;

  /// state_topic · command_service 는 상대 이름 (로봇 네임스페이스).
  explicit ZoneAgentLink(
    const RosNodeParams & params,
    std::chrono::milliseconds max_age = std::chrono::milliseconds(3000),
    const std::string & state_topic = "fleet_agent/state",
    const std::string & command_service = "fleet_agent/zone_command",
    std::chrono::milliseconds future_tolerance = std::chrono::milliseconds(500))
  : params_(params), max_age_(max_age), future_tolerance_(future_tolerance)
  {
    auto node = params_.node.lock();
    if (!node) {
      throw std::runtime_error("ZoneAgentLink: rclcpp 노드가 없다");
    }
    clock_ = node->get_clock();
    logger_ = node->get_logger();
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    std::weak_ptr<Shared> weak = shared_;
    sub_ = node->create_subscription<State>(
      state_topic, qos, [weak](State::SharedPtr msg) {
        if (auto s = weak.lock()) {
          std::lock_guard<std::mutex> lock(s->mutex);
          s->state = msg;
        }
      });
    client_ = params_.clients->template service<Command>(command_service);
  }

  /// 노드 시계 지금 (use_sim_time 이면 sim). /clock 전이면 0.
  rclcpp::Time now() const {return clock_->now();}

  /// 신선한 상태: −future_tolerance ≤ age ≤ max_age (없거나 범위 밖이면 nullptr).
  /// 시각은 노드 시계 (sim).
  State::SharedPtr freshState() const
  {
    State::SharedPtr st;
    {
      std::lock_guard<std::mutex> lock(shared_->mutex);
      st = shared_->state;
    }
    if (!st) {
      return nullptr;
    }
    const rclcpp::Time stamp(st->stamp, clock_->get_clock_type());
    const auto age = clock_->now() - stamp;
    if (age > rclcpp::Duration(max_age_) || age < rclcpp::Duration(-future_tolerance_)) {
      return nullptr;
    }
    return st;
  }

  /// 자원 전부가 허가돼 있나 (신선한 상태 기준).
  bool holdsAll(const std::vector<std::string> & resources) const
  {
    auto s = freshState();
    if (!s || resources.empty()) {
      return false;
    }
    for (const auto & r : resources) {
      if (std::find(s->held.begin(), s->held.end(), r) == s->held.end()) {
        return false;
      }
    }
    return true;
  }

  /// 작업 잠금이 거부됐나 (다른 로봇이 확정).
  bool isDenied(const std::string & resource) const
  {
    auto s = freshState();
    return s && std::find(s->denied.begin(), s->denied.end(), resource) != s->denied.end();
  }

  /// 진행 중 요청에 자원이 모두 들어 있나.
  bool requestedAll(const std::vector<std::string> & resources) const
  {
    auto s = freshState();
    if (!s) {
      return false;
    }
    for (const auto & r : resources) {
      if (std::find(s->requested.begin(), s->requested.end(), r) == s->requested.end()) {
        return false;
      }
    }
    return true;
  }

  bool serviceReady() const {return client_->service_is_ready();}

  /// 명령을 보낸다 (블로킹 없음). 응답은 token 으로 lastResponse() 에서 본다. 서버가 없으면 0.
  uint64_t send(uint8_t op, const std::vector<std::string> & resources, int32_t priority = 0)
  {
    if (!client_->service_is_ready()) {
      return 0;
    }
    auto req = std::make_shared<Command::Request>();
    req->op = op;
    req->resources = resources;
    req->priority = priority;
    uint64_t token = 0;
    {
      std::lock_guard<std::mutex> lock(shared_->mutex);
      token = ++shared_->next_token;
    }
    std::weak_ptr<Shared> weak = shared_;
    client_->async_send_request(
      req, [weak, token](rclcpp::Client<Command>::SharedFuture future) {
        if (auto s = weak.lock()) {
          std::lock_guard<std::mutex> lock(s->mutex);
          s->responses[token] = *future.get();
          while (s->responses.size() > 64) {
            s->responses.erase(s->responses.begin());
          }
        }
      });
    return token;
  }

  /// 응답 (아직 없으면 false).
  bool lastResponse(uint64_t token, Command::Response & out) const
  {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    auto it = shared_->responses.find(token);
    if (it == shared_->responses.end()) {
      return false;
    }
    out = it->second;
    return true;
  }

  const rclcpp::Logger & logger() const {return logger_;}

private:
  /// 콜백이 링크보다 오래 살 수 있어 (실행기 스레드) 공유 상태는 weak_ptr 로 넘긴다.
  struct Shared
  {
    std::mutex mutex;
    State::SharedPtr state;
    uint64_t next_token{0};
    std::map<uint64_t, Command::Response> responses;
  };

  RosNodeParams params_;
  std::chrono::milliseconds max_age_;
  std::chrono::milliseconds future_tolerance_;
  std::shared_ptr<Shared> shared_{std::make_shared<Shared>()};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("amr_behavior")};
  rclcpp::Subscription<State>::SharedPtr sub_;
  rclcpp::Client<Command>::SharedPtr client_;
};

using ZoneAgentLinkPtr = std::shared_ptr<ZoneAgentLink>;

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__ZONE_AGENT_LINK_HPP_
