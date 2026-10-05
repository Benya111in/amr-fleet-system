// 작업 BT 노드 등록 표 (단일 출처).
//
// 새 BT 노드 추가 절차 (명세 9장 "기존 코드 수정 3개 파일 이내"):
//   1) include/amr_behavior/bt_nodes/<node>.hpp  — 노드 클래스 (새 파일)
//   2) 등록 한 줄 — ROS 액션/서비스 노드는 아래 forEachRosNode(), 그 외는 src/bt_registry.cpp
//   3) behavior_trees/*.xml — 트리에서 사용
// 실행기(task_executor_node.cpp)와 테스트는 이 표를 그대로 쓰므로 수정할 필요가 없다.
#ifndef AMR_BEHAVIOR__BT_REGISTRY_HPP_
#define AMR_BEHAVIOR__BT_REGISTRY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "amr_behavior/bt_nodes/back_up.hpp"
#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "amr_behavior/bt_nodes/clear_costmap.hpp"
#include "amr_behavior/bt_nodes/dock.hpp"
#include "amr_behavior/bt_nodes/navigate_to_pose.hpp"
#include "amr_behavior/bt_nodes/ros_node_params.hpp"
#include "amr_behavior/bt_nodes/spin.hpp"
#include "amr_behavior/bt_nodes/wait.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace amr_behavior
{

/// ROS 액션/서비스 BT 노드 표. visitor.visit<Class>(ID) 를 노드마다 한 번 호출한다.
/// 실행기는 실제 클라이언트를, 테스트는 같은 포트의 스텁을 등록한다.
template<class Visitor>
void forEachRosNode(Visitor && visitor)
{
  visitor.template visit<NavigateToPoseAction>("NavigateToPose");
  visitor.template visit<DockAction>("Dock");
  visitor.template visit<SpinAction>("Spin");
  visitor.template visit<BackUpAction>("BackUp");
  visitor.template visit<WaitAction>("Wait");
  visitor.template visit<ClearCostmapService>("ClearCostmap");
}

/// 생성자 추가 인자(arg)를 붙여 노드를 만드는 빌더를 등록한다.
template<class T, class Arg>
void registerWithArg(BT::BehaviorTreeFactory & factory, const std::string & id, Arg arg)
{
  factory.registerBuilder<T>(
    id, [arg](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<T>(name, config, arg);
    });
}

/// 컨텍스트 노드(조건·단계·작업 보고·가상 적재·충전)와 자체 제어/데코레이터를 등록한다 (ROS
/// 불필요).
void registerCoreNodes(BT::BehaviorTreeFactory & factory, const ContextPtr & ctx);

/// ROS 액션/서비스 노드를 실제 클라이언트로 등록한다.
void registerRosNodes(BT::BehaviorTreeFactory & factory, const RosNodeParams & params);

/// 팩토리에 등록된 노드 중 BT.CPP 내장이 아닌 것의 ID (정렬).
std::vector<std::string> customNodeIds(const BT::BehaviorTreeFactory & factory);

/// XML 파일(들)에서 쓰인 노드 ID 집합 (<include> 를 따라간다). 서브트리 ID 와 루트 태그는 제외.
std::vector<std::string> nodeIdsUsedInFile(const std::string & xml_path);

/// ROS 액션/서비스 노드를 포트 정보(manifest)만으로 등록한다 (Groot 모델 생성용, 인스턴스화하면
/// 예외).
void registerRosNodeManifests(BT::BehaviorTreeFactory & factory);

/// Groot 팔레트: 내장이 아닌 노드의 <TreeNodesModel> 블록.
std::string treeNodesModelXml(const BT::BehaviorTreeFactory & factory);

/// 메인 XML 과 <include> 파일의 BehaviorTree 들을 한 파일로 합치고 TreeNodesModel 을 붙인다
/// (Groot 편집기는 include 를 따라가지 않으므로 문서·시각화용 단일 파일을 만든다).
std::string flattenTreeXml(const std::string & main_xml, const BT::BehaviorTreeFactory & factory);

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_REGISTRY_HPP_
