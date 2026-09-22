// Control ResumableSequence: 진행 위치를 블랙보드(progress_key)에 저장하는 Sequence.
//
// BT.CPP v3 의 SequenceStar 는 halt() 에서 인덱스를 0 으로 되돌리므로, E-stop 으로 작업 트리가
// halt 된 뒤 재개하면 이미 끝낸 단계(적재 등)를 다시 수행한다. 이 노드는 halt 되어도 저장된
// 단계부터 다시 시작한다.
//   - 자식 SUCCESS → 다음 단계, 진행 위치 저장
//   - 자식 FAILURE → 진행 위치 0 으로 초기화 후 FAILURE
//   - 모두 SUCCESS → 진행 위치 0 으로 초기화 후 SUCCESS
//   - halt() → 자식만 halt, 진행 위치는 유지 (재개 지점)
// 새 작업을 수락하면 실행기가 progress_key 를 0 으로 초기화한다.
#ifndef AMR_BEHAVIOR__BT_NODES__RESUMABLE_SEQUENCE_HPP_
#define AMR_BEHAVIOR__BT_NODES__RESUMABLE_SEQUENCE_HPP_

#include <string>

#include "behaviortree_cpp_v3/control_node.h"

namespace amr_behavior
{

class ResumableSequence : public BT::ControlNode
{
public:
  ResumableSequence(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>(
        "progress_key", "task_step", "진행 단계(int)를 저장할 블랙보드 키")};
  }

  /// 블랙보드의 진행 위치. 없거나 해석할 수 없으면 0.
  static int readProgress(const BT::Blackboard & bb, const std::string & key)
  {
    const BT::Any * any = bb.getAny(key);
    if (any == nullptr || any->empty()) {
      return 0;
    }
    try {
      if (any->type() == typeid(std::string)) {
        return BT::convertFromString<int>(any->cast<std::string>());
      }
      return any->cast<int>();
    } catch (const std::exception &) {
      return 0;
    }
  }

  BT::NodeStatus tick() override
  {
    std::string key = "task_step";
    getInput("progress_key", key);
    auto bb = config().blackboard;
    const int n = static_cast<int>(childrenCount());
    int index = readProgress(*bb, key);
    if (index < 0 || index >= n) {
      index = 0;
    }
    setStatus(BT::NodeStatus::RUNNING);
    while (index < n) {
      const BT::NodeStatus child = children_nodes_[index]->executeTick();
      if (child == BT::NodeStatus::RUNNING) {
        bb->set<int>(key, index);
        return BT::NodeStatus::RUNNING;
      }
      if (child == BT::NodeStatus::FAILURE) {
        haltChildren();
        bb->set<int>(key, 0);
        return BT::NodeStatus::FAILURE;
      }
      if (child == BT::NodeStatus::IDLE) {
        throw BT::LogicError("ResumableSequence: 자식 노드가 IDLE 을 반환했다");
      }
      ++index;   // SUCCESS → 다음 단계
      bb->set<int>(key, index);
    }
    haltChildren();
    bb->set<int>(key, 0);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__RESUMABLE_SEQUENCE_HPP_
