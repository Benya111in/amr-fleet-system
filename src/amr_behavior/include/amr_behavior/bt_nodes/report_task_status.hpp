// Action ReportTaskStatus: task_status(amr_msgs/Task) 를 발행한다.
// COMPLETED/FAILED 는 현재 작업을 끝낸다(다음 assign_task 수락 가능). 작업이 없으면 FAILURE.
#ifndef AMR_BEHAVIOR__BT_NODES__REPORT_TASK_STATUS_HPP_
#define AMR_BEHAVIOR__BT_NODES__REPORT_TASK_STATUS_HPP_

#include <cstdint>
#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "amr_msgs/msg/task.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace amr_behavior
{

class ReportTaskStatus : public BT::SyncActionNode
{
public:
  ReportTaskStatus(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("status", "PENDING / IN_PROGRESS / COMPLETED / FAILED"),
      BT::InputPort<std::string>("reason", "", "실패 사유 (로그용; Task.msg 에 필드 없음)"),
    };
  }

  /// 상태 문자열 → Task 상수. 모르는 값이면 false.
  static bool parseStatus(const std::string & text, uint8_t & status)
  {
    using amr_msgs::msg::Task;
    if (text == "PENDING") {
      status = Task::STATUS_PENDING;
    } else if (text == "IN_PROGRESS") {
      status = Task::STATUS_IN_PROGRESS;
    } else if (text == "COMPLETED") {
      status = Task::STATUS_COMPLETED;
    } else if (text == "FAILED") {
      status = Task::STATUS_FAILED;
    } else {
      return false;
    }
    return true;
  }

  BT::NodeStatus tick() override
  {
    const std::string text = inputOr<std::string>(*this, "status", "");
    uint8_t status = 0;
    if (!parseStatus(text, status)) {
      ctx_->logWarn("ReportTaskStatus: 알 수 없는 상태 '" + text + "'");
      return BT::NodeStatus::FAILURE;
    }
    const std::string reason = inputOr<std::string>(*this, "reason", "");
    return ctx_->reportStatus(status, reason) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  ContextPtr ctx_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__REPORT_TASK_STATUS_HPP_
