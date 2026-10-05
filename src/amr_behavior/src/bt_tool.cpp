// bt_tool: Groot 용 파일 생성기 (ROS 실행 불필요).
//   ros2 run amr_behavior bt_tool models              → 자체 노드 팔레트 (<TreeNodesModel>)
//   ros2 run amr_behavior bt_tool flatten [main.xml]  → include 를 합친 단일 트리 + 팔레트
//                                                       (Groot 편집기용)
//   ros2 run amr_behavior bt_tool nodes [main.xml]    → 트리에 쓰인 노드 타입 목록 (분류별)
#include <iostream>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "amr_behavior/bt_registry.hpp"
#include "amr_behavior/executor_context.hpp"

int main(int argc, char ** argv)
{
  const std::string mode = argc > 1 ? argv[1] : "";
  std::string xml = argc > 2 ? argv[2] : "";
  if (mode != "models" && mode != "flatten" && mode != "nodes") {
    std::cerr << "usage: bt_tool models | flatten [main.xml] | nodes [main.xml]\n";
    return 2;
  }
  if (xml.empty()) {
    xml = ament_index_cpp::get_package_share_directory("amr_behavior") +
      "/behavior_trees/task_executor.xml";
  }
  BT::BehaviorTreeFactory factory;
  auto ctx = std::make_shared<amr_behavior::ExecutorContext>([]() {return 0.0;});
  amr_behavior::registerCoreNodes(factory, ctx);
  amr_behavior::registerRosNodeManifests(factory);
  try {
    if (mode == "models") {
      std::cout << amr_behavior::treeNodesModelXml(factory) << "\n";
    } else if (mode == "flatten") {
      std::cout << amr_behavior::flattenTreeXml(xml, factory);
    } else {
      for (const auto & id : amr_behavior::nodeIdsUsedInFile(xml)) {
        const auto it = factory.manifests().find(id);
        const std::string type = it == factory.manifests().end() ?
          "?" : BT::toStr(it->second.type);
        const bool custom = factory.builtinNodes().count(id) == 0U;
        std::cout << type << "\t" << id << (custom ? "\t(자체)" : "") << "\n";
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "bt_tool: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
