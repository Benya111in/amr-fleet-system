// BT 시험 도구: ROS 액션/서비스 노드를 같은 ID·포트의 스텁으로 바꿔 등록한다.
//
// 스텁은 호출마다 대본(outcomes)에서 결과를 하나 꺼내고, running_ticks 동안 RUNNING 을 낸 뒤 그
// 결과를 낸다. 호출·halt 횟수와 시작 시 입력 포트 값(goal 좌표, dock_id 등)을 기록해 복구 경로를
// 검증한다.
#ifndef BT_TEST_UTILS_HPP_
#define BT_TEST_UTILS_HPP_

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "amr_behavior/bt_nodes/bt_conversions.hpp"
#include "amr_behavior/bt_registry.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace amr_behavior_test
{

struct StubScript
{
  std::deque<BT::NodeStatus> outcomes;                      ///< 앞에서부터 한 번씩 소비
  BT::NodeStatus fallback{BT::NodeStatus::SUCCESS};          ///< 대본이 비면 이 결과
  int running_ticks{1};                                      ///< 결과 전 RUNNING tick 수
  int calls{0};
  int halts{0};
  std::vector<std::string> starts;                           ///< 시작 시 입력 요약

  void script(std::initializer_list<BT::NodeStatus> list) {outcomes = list;}
};

class StubAction : public BT::ActionNodeBase
{
public:
  StubAction(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<StubScript> script)
  : BT::ActionNodeBase(name, config), script_(std::move(script)) {}

  BT::NodeStatus tick() override
  {
    if (status() == BT::NodeStatus::IDLE) {
      setStatus(BT::NodeStatus::RUNNING);
      ++script_->calls;
      script_->starts.push_back(describeInputs());
      remaining_ = script_->running_ticks;
      if (script_->outcomes.empty()) {
        outcome_ = script_->fallback;
      } else {
        outcome_ = script_->outcomes.front();
        script_->outcomes.pop_front();
      }
    }
    if (remaining_ > 0) {
      --remaining_;
      return BT::NodeStatus::RUNNING;
    }
    return outcome_;
  }

  void halt() override
  {
    if (status() == BT::NodeStatus::RUNNING) {
      ++script_->halts;
    }
    setStatus(BT::NodeStatus::IDLE);
  }

private:
  std::string describeInputs()
  {
    std::string out;
    geometry_msgs::msg::PoseStamped goal;
    if (getInput("goal", goal)) {
      out += "goal=" + goal.header.frame_id + ":" + std::to_string(goal.pose.position.x) + "," +
        std::to_string(goal.pose.position.y);
    }
    std::string text;
    if (getInput("dock_id", text)) {
      out += "dock_id=" + text;
    }
    if (getInput("service_name", text)) {
      out += "service=" + text;
    }
    double value = 0.0;
    if (getInput("spin_dist", value)) {
      out += "spin=" + std::to_string(value);
    }
    if (getInput("backup_dist", value)) {
      out += "backup=" + std::to_string(value);
    }
    return out;
  }

  std::shared_ptr<StubScript> script_;
  int remaining_{0};
  BT::NodeStatus outcome_{BT::NodeStatus::SUCCESS};
};

/// forEachRosNode 방문자: 실제 노드의 포트 목록으로 스텁을 등록한다.
struct StubRegistrar
{
  BT::BehaviorTreeFactory & factory;
  std::map<std::string, std::shared_ptr<StubScript>> & scripts;

  template<class T>
  void visit(const std::string & id)
  {
    auto script = std::make_shared<StubScript>();
    scripts[id] = script;
    BT::TreeNodeManifest manifest;
    manifest.type = BT::getType<T>();
    manifest.registration_ID = id;
    manifest.ports = T::providedPorts();
    factory.registerBuilder(
      manifest, [script](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<StubAction>(name, config, script);
      });
  }
};

inline std::map<std::string, std::shared_ptr<StubScript>> registerStubRosNodes(
  BT::BehaviorTreeFactory & factory)
{
  std::map<std::string, std::shared_ptr<StubScript>> scripts;
  amr_behavior::forEachRosNode(StubRegistrar{factory, scripts});
  return scripts;
}

}  // namespace amr_behavior_test

#endif  // BT_TEST_UTILS_HPP_
