// Action CommitTaskLock: 픽업 시작 직전에 작업 ID 잠금을 확정한다 (보너스 4 P3).
//
// COMMIT_TASK 를 보내고 task:<id> 가 held 가 될 때 (과반 ENTER_ACK) SUCCESS. 에이전트가 거부
// (다른 로봇이 확정 — denied) 하면 FAILURE: 이 로봇은 픽업하지 않는다 (중복 실행 0 의 실행기
// 쪽 절반). timeout_ms(노드 시계 = sim, P3 리뷰 §8-14)를 넘겨도 FAILURE. 재요청은 벽시계 1 s 마다.
// 확정 잠금은 작업이 끝나도 풀지 않는다 (에이전트 몫).
#ifndef AMR_BEHAVIOR__BT_NODES__ZONE_TASK_COMMIT_HPP_
#define AMR_BEHAVIOR__BT_NODES__ZONE_TASK_COMMIT_HPP_

#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include "amr_behavior/bt_nodes/zone_agent_link.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace amr_behavior
{

class CommitTaskLock : public BT::StatefulActionNode
{
public:
  CommitTaskLock(
    const std::string & name, const BT::NodeConfiguration & config, ZoneAgentLinkPtr link)
  : BT::StatefulActionNode(name, config), link_(std::move(link)) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("task_id", "작업 ID"),
      BT::InputPort<unsigned>(
        "timeout_ms", 30000, "확정 대기 상한 [ms, 노드 시계 = sim] (0 = 없음)"),
    };
  }

  BT::NodeStatus onStart() override
  {
    std::string id;
    if (!getInput("task_id", id) || id.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    resource_ = {"task:" + id};
    unsigned t = 30000;
    getInput("timeout_ms", t);
    timeout_ = std::chrono::milliseconds(t);
    watch_.start(link_->now());
    return check(true);
  }

  BT::NodeStatus onRunning() override {return check(false);}

  void onHalted() override {}

private:
  using Clock = std::chrono::steady_clock;

  BT::NodeStatus check(bool first)
  {
    if (link_->holdsAll(resource_)) {
      return BT::NodeStatus::SUCCESS;
    }
    if (link_->isDenied(resource_.front())) {
      RCLCPP_WARN(
        link_->logger(), "[%s] 작업 잠금 거부: %s", name().c_str(),
        resource_.front().c_str());
      return BT::NodeStatus::FAILURE;
    }
    if (timeout_.count() > 0 &&
      watch_.elapsedS(link_->now()) * 1000.0 > static_cast<double>(timeout_.count()))
    {
      RCLCPP_WARN(
        link_->logger(), "[%s] 작업 잠금 확정 시간 초과 (%u ms, sim): %s", name().c_str(),
        static_cast<unsigned>(timeout_.count()), resource_.front().c_str());
      return BT::NodeStatus::FAILURE;
    }
    const auto now = Clock::now();
    if (first || now - last_send_ > std::chrono::seconds(1)) {
      // 멱등: 에이전트는 확정 표시만 다시 세운다
      link_->send(ZoneAgentLink::Command::Request::OP_COMMIT_TASK, resource_);
      last_send_ = now;
    }
    return BT::NodeStatus::RUNNING;
  }

  ZoneAgentLinkPtr link_;
  std::vector<std::string> resource_;
  std::chrono::milliseconds timeout_{30000};
  SimStopwatch watch_;
  Clock::time_point last_send_{};
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__ZONE_TASK_COMMIT_HPP_
