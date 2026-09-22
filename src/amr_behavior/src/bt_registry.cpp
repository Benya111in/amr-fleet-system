// 작업 BT 노드 등록 표 구현 (bt_registry.hpp 설명 참고).
#include "amr_behavior/bt_registry.hpp"

#include <algorithm>
#include <fstream>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "amr_behavior/bt_nodes/is_battery_ok.hpp"
#include "amr_behavior/bt_nodes/is_dock_marker_visible.hpp"
#include "amr_behavior/bt_nodes/is_estop_clear.hpp"
#include "amr_behavior/bt_nodes/is_localized.hpp"
#include "amr_behavior/bt_nodes/is_object_detected.hpp"
#include "amr_behavior/bt_nodes/is_task_assigned.hpp"
#include "amr_behavior/bt_nodes/is_traffic_hold.hpp"
#include "amr_behavior/bt_nodes/keep_running_until_success.hpp"
#include "amr_behavior/bt_nodes/report_task_status.hpp"
#include "amr_behavior/bt_nodes/resumable_sequence.hpp"
#include "amr_behavior/bt_nodes/set_charging.hpp"
#include "amr_behavior/bt_nodes/set_fail_reason.hpp"
#include "amr_behavior/bt_nodes/set_phase.hpp"
#include "amr_behavior/bt_nodes/simulate_payload.hpp"
#include "behaviortree_cpp_v3/xml_parsing.h"

namespace amr_behavior
{

void registerCoreNodes(BT::BehaviorTreeFactory & factory, const ContextPtr & ctx)
{
  // Condition
  registerWithArg<IsTaskAssigned>(factory, "IsTaskAssigned", ctx);
  registerWithArg<IsBatteryOk>(factory, "IsBatteryOk", ctx);
  registerWithArg<IsEstopClear>(factory, "IsEstopClear", ctx);
  registerWithArg<IsTrafficHold>(factory, "IsTrafficHold", ctx);
  registerWithArg<IsLocalized>(factory, "IsLocalized", ctx);
  registerWithArg<IsObjectDetected>(factory, "IsObjectDetected", ctx);
  registerWithArg<IsDockMarkerVisible>(factory, "IsDockMarkerVisible", ctx);
  // Action (ROS 불필요: 컨텍스트 훅으로 발행)
  registerWithArg<SimulateLoad>(factory, "SimulateLoad", ctx);
  registerWithArg<SimulateUnload>(factory, "SimulateUnload", ctx);
  registerWithArg<ReportTaskStatus>(factory, "ReportTaskStatus", ctx);
  registerWithArg<SetPhase>(factory, "SetPhase", ctx);
  registerWithArg<SetCharging>(factory, "SetCharging", ctx);
  // Control / Decorator
  factory.registerNodeType<ResumableSequence>("ResumableSequence");
  factory.registerNodeType<KeepRunningUntilSuccess>("KeepRunningUntilSuccess");
  factory.registerNodeType<SetFailReason>("SetFailReason");
}

namespace
{
struct RosRegistrar
{
  BT::BehaviorTreeFactory & factory;
  const RosNodeParams & params;

  template<class T>
  void visit(const std::string & id)
  {
    registerWithArg<T>(factory, id, params);
  }
};

std::string readFile(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("BT XML 을 열 수 없다: " + path);
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string directoryOf(const std::string & path)
{
  const auto pos = path.find_last_of('/');
  return pos == std::string::npos ? std::string(".") : path.substr(0, pos);
}

void collectIds(const std::string & path, std::set<std::string> & ids, std::set<std::string> & seen)
{
  if (!seen.insert(path).second) {
    return;
  }
  std::string text = readFile(path);
  // 주석 제거 (주석 안의 예시 태그를 세지 않도록)
  text = std::regex_replace(text, std::regex("<!--[\\s\\S]*?-->"), "");
  static const std::set<std::string> kStructural = {
    "root", "BehaviorTree", "include", "TreeNodesModel", "Action", "Condition", "Control",
    "Decorator", "SubTree", "input_port", "output_port", "inout_port"};
  const std::regex tag("<([A-Za-z_][A-Za-z0-9_]*)");
  for (auto it = std::sregex_iterator(text.begin(), text.end(), tag); it != std::sregex_iterator();
    ++it)
  {
    const std::string name = (*it)[1].str();
    if (kStructural.count(name) == 0U) {
      ids.insert(name);
    }
  }
  const std::regex include("<include\\s+path=\"([^\"]+)\"");
  for (auto it = std::sregex_iterator(text.begin(), text.end(), include);
    it != std::sregex_iterator(); ++it)
  {
    std::string child = (*it)[1].str();
    if (child.empty() || child[0] != '/') {
      child = directoryOf(path) + "/" + child;
    }
    collectIds(child, ids, seen);
  }
}
}  // namespace

void registerRosNodes(BT::BehaviorTreeFactory & factory, const RosNodeParams & params)
{
  forEachRosNode(RosRegistrar{factory, params});
}

std::vector<std::string> customNodeIds(const BT::BehaviorTreeFactory & factory)
{
  std::vector<std::string> ids;
  const auto & builtin = factory.builtinNodes();
  for (const auto & entry : factory.manifests()) {
    if (builtin.count(entry.first) == 0U) {
      ids.push_back(entry.first);
    }
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

std::vector<std::string> nodeIdsUsedInFile(const std::string & xml_path)
{
  std::set<std::string> ids;
  std::set<std::string> seen;
  collectIds(xml_path, ids, seen);
  return {ids.begin(), ids.end()};
}

namespace
{
struct ManifestRegistrar
{
  BT::BehaviorTreeFactory & factory;

  template<class T>
  void visit(const std::string & id)
  {
    BT::TreeNodeManifest manifest;
    manifest.type = BT::getType<T>();
    manifest.registration_ID = id;
    manifest.ports = T::providedPorts();
    factory.registerBuilder(
      manifest, [id](const std::string &, const BT::NodeConfiguration &)
      -> std::unique_ptr<BT::TreeNode> {
        throw BT::RuntimeError("노드 모델 전용 등록이다 (실행 불가): " + id);
      });
  }
};

/// text 에서 <tag ...> ... </tag> 블록들을 순서대로 뽑는다.
std::vector<std::string> extractBlocks(const std::string & text, const std::string & tag)
{
  std::vector<std::string> blocks;
  const std::regex re("<" + tag + "[\\s>][\\s\\S]*?</" + tag + ">");
  for (auto it = std::sregex_iterator(text.begin(), text.end(), re); it != std::sregex_iterator();
    ++it)
  {
    blocks.push_back(it->str());
  }
  return blocks;
}

void collectTrees(
  const std::string & path, std::vector<std::string> & trees,
  std::set<std::string> & seen)
{
  if (!seen.insert(path).second) {
    return;
  }
  const std::string text = std::regex_replace(
    readFile(path), std::regex("<!--[\\s\\S]*?-->"), "");
  for (auto & block : extractBlocks(text, "BehaviorTree")) {
    trees.push_back(block);
  }
  const std::regex include("<include\\s+path=\"([^\"]+)\"");
  for (auto it = std::sregex_iterator(text.begin(), text.end(), include);
    it != std::sregex_iterator(); ++it)
  {
    std::string child = (*it)[1].str();
    if (child.empty() || child[0] != '/') {
      child = directoryOf(path) + "/" + child;
    }
    collectTrees(child, trees, seen);
  }
}
}  // namespace

void registerRosNodeManifests(BT::BehaviorTreeFactory & factory)
{
  forEachRosNode(ManifestRegistrar{factory});
}

std::string treeNodesModelXml(const BT::BehaviorTreeFactory & factory)
{
  const auto blocks = extractBlocks(BT::writeTreeNodesModelXML(factory), "TreeNodesModel");
  return blocks.empty() ? std::string("<TreeNodesModel/>") : blocks.front();
}

std::string flattenTreeXml(const std::string & main_xml, const BT::BehaviorTreeFactory & factory)
{
  const std::string text = readFile(main_xml);
  std::smatch main_id;
  std::string main_tree;
  if (std::regex_search(text, main_id, std::regex("main_tree_to_execute=\"([^\"]+)\""))) {
    main_tree = main_id[1].str();
  }
  std::vector<std::string> trees;
  std::set<std::string> seen;
  collectTrees(main_xml, trees, seen);
  std::string out = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  out += "<!-- 생성 파일: ros2 run amr_behavior bt_tool flatten (직접 수정하지 말 것) -->\n";
  out += "<root main_tree_to_execute=\"" + main_tree + "\">\n";
  for (const auto & tree : trees) {
    out += "  " + tree + "\n";
  }
  out += "  " + treeNodesModelXml(factory) + "\n</root>\n";
  return out;
}

}  // namespace amr_behavior
