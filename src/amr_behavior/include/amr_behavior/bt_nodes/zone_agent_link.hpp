// 분산 플릿 에이전트 연결 (보너스 4 P3): fleet_agent/state 구독 + zone_command 클라이언트.
//
// 허가의 근거는 에이전트 상태의 held 하나뿐이다 (허가 목록, fail-closed).
// 상태가 max_age 보다 오래되면 (에이전트가 죽었거나 끊김) 아무것도 허가되지 않은 것으로 본다.
// 모든 존 BT 노드가 이 연결 하나를 나눠 쓴다 (registerZoneNodes 가 만든다).
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
    const std::string & command_service = "fleet_agent/zone_command")
  : params_(params), max_age_(max_age)
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

  /// 신선한 상태 (없거나 max_age 보다 오래되면 nullptr). 시각은 노드 시계 (sim).
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
    if (age > rclcpp::Duration(max_age_)) {
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
  std::shared_ptr<Shared> shared_{std::make_shared<Shared>()};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("amr_behavior")};
  rclcpp::Subscription<State>::SharedPtr sub_;
  rclcpp::Client<Command>::SharedPtr client_;
};

using ZoneAgentLinkPtr = std::shared_ptr<ZoneAgentLink>;

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__ZONE_AGENT_LINK_HPP_
