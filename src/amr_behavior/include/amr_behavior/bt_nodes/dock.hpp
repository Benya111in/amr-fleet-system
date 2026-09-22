// Action Dock: docking_server_node 의 dock 액션 (amr_msgs/action/Dock) 클라이언트.
//
// dock_id 가 비어 있으면(작업 자세가 등록 도크와 맞지 않음) 도킹을 생략하고 SUCCESS.
// 결과(성공/실패 모두)의 최종 오차·시도 횟수를 출력 포트로 내보낸다 → 로그·실패 사유.
// max_retries 는 이 goal 한 번에 서버가 수행할 최대 접근 횟수 (0 = 서버 기본값).
// BT 는 DockAt 서브트리의 RetryUntilSuccessful 로 goal 을 다시 보내므로 기본 1 (단일 재시도
// 카운터).
#ifndef AMR_BEHAVIOR__BT_NODES__DOCK_HPP_
#define AMR_BEHAVIOR__BT_NODES__DOCK_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_conversions.hpp"
#include "amr_behavior/bt_nodes/ros_action_node.hpp"
#include "amr_msgs/action/dock.hpp"

namespace amr_behavior
{

class DockAction : public RosActionNode<amr_msgs::action::Dock>
{
public:
  DockAction(const std::string & name, const BT::NodeConfiguration & config, RosNodeParams params)
  : RosActionNode(name, config, std::move(params), "dock") {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("dock_id", "도크 id (\"\" = 도킹 생략)"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "approach_pose", "도킹 시작(staging) 자세 (map, 선택)"),
        BT::InputPort<unsigned>("max_retries", 1, "이 goal 의 최대 접근 횟수 (0 = 서버 기본)"),
        BT::OutputPort<unsigned>("attempts_used", "서버가 쓴 접근 횟수"),
        BT::OutputPort<double>("final_position_error", "최종 위치 오차 [m]"),
        BT::OutputPort<double>("final_angle_error", "최종 각도 오차 [rad]"),
        BT::OutputPort<std::string>("dock_phase", "피드백: search/align/approach/final/backup"),
      });
  }

protected:
  bool skipGoal() override
  {
    std::string id;
    getInput("dock_id", id);
    if (id.empty()) {
      RCLCPP_INFO(logger_, "[%s] dock_id 없음 → 도킹 생략", name().c_str());
      return true;
    }
    return false;
  }

  bool setGoal(Goal & goal) override
  {
    if (!getInput("dock_id", goal.dock_id) || goal.dock_id.empty()) {
      return false;
    }
    getInput("approach_pose", goal.approach_pose);
    unsigned retries = 1;
    getInput("max_retries", retries);
    goal.max_retries = static_cast<uint8_t>(retries > 255U ? 255U : retries);
    return true;
  }

  void onFeedback(const Feedback & feedback) override
  {
    setOutput("dock_phase", feedback.current_phase);
  }

  BT::NodeStatus onResult(const WrappedResult & result) override
  {
    if (result.result) {
      setOutput("attempts_used", static_cast<unsigned>(result.result->attempts_used));
      setOutput("final_position_error", static_cast<double>(result.result->final_position_error));
      setOutput("final_angle_error", static_cast<double>(result.result->final_angle_error));
    }
    const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
      result.result->success;
    if (!ok) {
      RCLCPP_WARN(
        logger_, "[%s] 도킹 실패 (code=%d, attempts=%u)", name().c_str(),
        static_cast<int>(result.code),
        result.result ? static_cast<unsigned>(result.result->attempts_used) : 0U);
    }
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__DOCK_HPP_
