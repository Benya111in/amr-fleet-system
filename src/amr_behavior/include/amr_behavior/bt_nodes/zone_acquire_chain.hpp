// Action AcquireZoneChain: 구역 사슬 전체의 진입 허가를 에이전트에 청하고, 과반 ENTER_ACK 로
// 허가(held)될 때까지 RUNNING (보너스 4 P3, 보고서 "결정점에서 멈춘 뒤 AcquireZoneChain").
//
// 전부-아니면-전무: zones 의 자원이 모두 held 에 있어야 SUCCESS. 에이전트가 거절하면
// (ISOLATED 등) FAILURE, timeout_ms 를 넘기면 CANCEL 을 보내고 FAILURE. halt 되면 CANCEL
// (잠정 표를 풀게). timeout_ms 는 노드 시계(sim) 기준이다 — 에이전트의 투표 재시도 · 백오프와
// 같은 축 (P3 리뷰 §8-14). 상태에 요청이 안 보이면 (에이전트 재기동 등) 벽시계 1 s 마다 다시
// 보낸다 (에이전트 쪽 멱등 — 재전송은 통신 현상이라 벽시계, p0.md §2 의 응답 시한과 같은 규칙).
#ifndef AMR_BEHAVIOR__BT_NODES__ZONE_ACQUIRE_CHAIN_HPP_
#define AMR_BEHAVIOR__BT_NODES__ZONE_ACQUIRE_CHAIN_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "amr_behavior/bt_nodes/zone_agent_link.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace amr_behavior
{

class AcquireZoneChain : public BT::StatefulActionNode
{
public:
  AcquireZoneChain(
    const std::string & name, const BT::NodeConfiguration & config, ZoneAgentLinkPtr link)
  : BT::StatefulActionNode(name, config), link_(std::move(link)) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("zones", "사슬 자원 (예: x_ab_1;x_ab_2 — 접두사 없으면 zone:)"),
      BT::InputPort<int>("priority", 0, "작업 우선순위 (투표 대기열 순서)"),
      BT::InputPort<unsigned>("timeout_ms", 0, "허가 대기 상한 [ms, 노드 시계 = sim] (0 = 없음)"),
    };
  }

  BT::NodeStatus onStart() override
  {
    std::string text;
    if (!getInput("zones", text)) {
      return BT::NodeStatus::FAILURE;
    }
    zones_ = parseResources(text);
    if (zones_.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    priority_ = 0;
    getInput("priority", priority_);
    unsigned t = 0;
    getInput("timeout_ms", t);
    timeout_ = std::chrono::milliseconds(t);
    watch_.start(link_->now());
    if (link_->holdsAll(zones_)) {
      return BT::NodeStatus::SUCCESS;
    }
    sendAcquire();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (link_->holdsAll(zones_)) {
      return BT::NodeStatus::SUCCESS;
    }
    ZoneAgentLink::Command::Response resp;
    if (token_ != 0 && link_->lastResponse(token_, resp) && !resp.accepted) {
      RCLCPP_WARN(
        link_->logger(), "[%s] 에이전트 거절: %s", name().c_str(),
        resp.message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    if (timeout_.count() > 0 &&
      watch_.elapsedS(link_->now()) * 1000.0 > static_cast<double>(timeout_.count()))
    {
      link_->send(ZoneAgentLink::Command::Request::OP_CANCEL, {});
      return BT::NodeStatus::FAILURE;
    }
    const auto now = Clock::now();
    if (now - last_send_ > std::chrono::seconds(1) && !link_->requestedAll(zones_)) {
      sendAcquire();
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    link_->send(ZoneAgentLink::Command::Request::OP_CANCEL, {});
  }

  const std::vector<std::string> & zones() const {return zones_;}

private:
  using Clock = std::chrono::steady_clock;

  void sendAcquire()
  {
    token_ = link_->send(ZoneAgentLink::Command::Request::OP_ACQUIRE, zones_, priority_);
    last_send_ = Clock::now();
  }

  ZoneAgentLinkPtr link_;
  std::vector<std::string> zones_;
  int priority_{0};
  std::chrono::milliseconds timeout_{0};
  SimStopwatch watch_;
  Clock::time_point last_send_{};
  uint64_t token_{0};
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__ZONE_ACQUIRE_CHAIN_HPP_
