// Fault-injection harness for mission.xml on BehaviorTree.CPP 4.10 (throwaway container only).
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <iostream>
#include <map>
#include <thread>
#include <vector>
using namespace BT;

struct G { int tasks = 0; bool estop = false, hold = false, nav_fail = false, dock_fail_pickup = false;
           double battery = 100; bool low_latch = false; std::vector<std::string> log; } g;
static void L(const std::string& s) { g.log.push_back(s); }
static int count(const std::string& s) { int n = 0; for (auto& e : g.log) n += (e == s); return n; }

#define SYNC(NAME, PORTS, ...) \
  class NAME : public SyncActionNode { public: NAME(const std::string& n, const NodeConfig& c) : SyncActionNode(n, c) {} \
    static PortsList providedPorts() { return PORTS; } NodeStatus tick() override { __VA_ARGS__ } };
#define COND(NAME, PORTS, ...) \
  class NAME : public ConditionNode { public: NAME(const std::string& n, const NodeConfig& c) : ConditionNode(n, c) {} \
    static PortsList providedPorts() { return PORTS; } NodeStatus tick() override { __VA_ARGS__ } };

class WaitForTask : public StatefulActionNode { public: using StatefulActionNode::StatefulActionNode;
  static PortsList providedPorts() { return {OutputPort<std::string>("task_id")}; }
  NodeStatus onStart() override { return onRunning(); }
  NodeStatus onRunning() override { if (g.tasks > 0) { setOutput("task_id", std::string("T1")); return NodeStatus::SUCCESS; } return NodeStatus::RUNNING; }
  void onHalted() override {} };
class Gate : public StatefulActionNode { public: Gate(const std::string& n, const NodeConfig& c, bool* f) : StatefulActionNode(n, c), f_(f) {}
  static PortsList providedPorts() { return {}; }
  NodeStatus onStart() override { return *f_ ? NodeStatus::RUNNING : NodeStatus::SUCCESS; }
  NodeStatus onRunning() override { return *f_ ? NodeStatus::RUNNING : NodeStatus::SUCCESS; }
  void onHalted() override {} bool* f_; };
class NavigateTo : public StatefulActionNode { public: using StatefulActionNode::StatefulActionNode; int k = 0; std::string goal;
  static PortsList providedPorts() { return {InputPort<std::string>("goal"), OutputPort<double>("eta"), OutputPort<std::string>("result")}; }
  NodeStatus onStart() override { getInput("goal", goal); L("NAV:" + goal); k = 0; return NodeStatus::RUNNING; }
  NodeStatus onRunning() override { if (++k < 4) return NodeStatus::RUNNING;
    if (g.nav_fail) { setOutput("result", std::string("NAV_FAILED")); return NodeStatus::FAILURE; } L("NAV_OK:" + goal); return NodeStatus::SUCCESS; }
  void onHalted() override { L("NAV_CANCEL:" + goal); } };
class DockToStation : public StatefulActionNode { public: using StatefulActionNode::StatefulActionNode; int k = 0; std::string id;
  static PortsList providedPorts() { return {InputPort<std::string>("dock_id"), InputPort<unsigned>("max_retries"), OutputPort<std::string>("result")}; }
  NodeStatus onStart() override { getInput("dock_id", id); L("DOCK:" + id); k = 0; return NodeStatus::RUNNING; }
  NodeStatus onRunning() override { if (++k < 3) return NodeStatus::RUNNING;
    if (g.dock_fail_pickup && id == "pickup") { setOutput("result", std::string("LATERAL_DEVIATION")); return NodeStatus::FAILURE; } return NodeStatus::SUCCESS; }
  void onHalted() override { L("DOCK_CANCEL:" + id); } };
class ChargeUntil : public StatefulActionNode { public: using StatefulActionNode::StatefulActionNode;
  static PortsList providedPorts() { return {InputPort<double>("level_pct")}; }
  NodeStatus onStart() override { L("CHARGE_START"); return NodeStatus::RUNNING; }
  NodeStatus onRunning() override { g.battery += 20; if (g.battery >= 80) { L("CHARGED"); return NodeStatus::SUCCESS; } return NodeStatus::RUNNING; }
  void onHalted() override {} };
COND(IsBatteryLow, (PortsList{InputPort<double>("enter_pct"), InputPort<double>("exit_pct")}),
     { double en = 20, ex = 80; getInput("enter_pct", en); getInput("exit_pct", ex);
       if (g.battery < en) g.low_latch = true; if (g.battery >= ex) g.low_latch = false;
       return g.low_latch ? NodeStatus::SUCCESS : NodeStatus::FAILURE; })
COND(HasPendingTask, (PortsList{}), { return g.tasks > 0 ? NodeStatus::SUCCESS : NodeStatus::FAILURE; })
COND(IsMarkerVisible, (PortsList{InputPort<std::string>("dock_id")}), { return NodeStatus::SUCCESS; })
COND(IsDeadlineAtRisk, (PortsList{OutputPort<double>("speed_limit")}), { return NodeStatus::FAILURE; })
SYNC(AcceptTask, (PortsList{InputPort<std::string>("task_id"), OutputPort<std::string>("pickup_staging"), OutputPort<std::string>("pickup_dock"),
     OutputPort<std::string>("dropoff_staging"), OutputPort<std::string>("dropoff_dock"), OutputPort<std::string>("item_type"),
     OutputPort<double>("item_mass"), OutputPort<unsigned>("load_time_ms"), OutputPort<unsigned>("budget_ms")}),
     { g.tasks--; L("ACCEPT"); setOutput("pickup_staging", std::string("pickup_staging")); setOutput("pickup_dock", std::string("pickup"));
       setOutput("dropoff_staging", std::string("dropoff_staging")); setOutput("dropoff_dock", std::string("dropoff"));
       setOutput("item_type", std::string("medium")); setOutput("item_mass", 10.0); setOutput("load_time_ms", 5u); setOutput("budget_ms", 600000u);
       return NodeStatus::SUCCESS; })
SYNC(SetSpeedLimit, (PortsList{InputPort<double>("limit")}), { return NodeStatus::SUCCESS; })
SYNC(ClearCostmaps, (PortsList{}), { L("REC:clear"); return NodeStatus::SUCCESS; })
SYNC(BackUp, (PortsList{InputPort<double>("dist")}), { L("REC:backup"); return NodeStatus::SUCCESS; })
SYNC(Spin, (PortsList{InputPort<double>("angle")}), { L("REC:spin"); return NodeStatus::SUCCESS; })
SYNC(SearchMarker, (PortsList{InputPort<std::string>("dock_id"), InputPort<double>("budget_s"), OutputPort<std::string>("result")}), { return NodeStatus::SUCCESS; })
SYNC(BackToStaging, (PortsList{InputPort<std::string>("dock_id")}), { L("BACK_TO_STAGING"); return NodeStatus::SUCCESS; })
SYNC(VirtualLoad, (PortsList{InputPort<std::string>("item_type")}), { L("LOAD"); return NodeStatus::SUCCESS; })
SYNC(VirtualUnload, (PortsList{}), { L("UNLOAD"); return NodeStatus::SUCCESS; })
SYNC(PublishTaskEvent, (PortsList{InputPort<std::string>("status"), InputPort<std::string>("reason")}),
     { std::string s, r; getInput("status", s); getInput("reason", r); L("EVENT:" + s); if (s == "FAILED") L("REASON:" + r); return NodeStatus::SUCCESS; })
SYNC(LogTaskRecord, (PortsList{}), { return NodeStatus::SUCCESS; })
SYNC(RequestReassignment, (PortsList{}), { L("REASSIGN"); return NodeStatus::SUCCESS; })

class RecoveryNode : public ControlNode { public: using ControlNode::ControlNode; size_t idx = 0; int retries = 0;
  static PortsList providedPorts() { return {InputPort<int>("number_of_retries")}; }
  NodeStatus tick() override { int nr = 1; getInput("number_of_retries", nr);
    if (status() == NodeStatus::IDLE) { idx = 0; retries = 0; } setStatus(NodeStatus::RUNNING);
    while (true) {
      if (idx == 0) { auto s = children_nodes_[0]->executeTick();
        if (s == NodeStatus::RUNNING) return s;
        if (s == NodeStatus::SUCCESS) { resetChildren(); retries = 0; return s; }
        if (retries < nr) { haltChild(0); idx = 1; continue; }
        resetChildren(); retries = 0; return NodeStatus::FAILURE;
      } else { auto s = children_nodes_[1]->executeTick();
        if (s == NodeStatus::RUNNING) return s;
        if (s == NodeStatus::SUCCESS) { retries++; haltChild(1); idx = 0; continue; }
        resetChildren(); idx = 0; retries = 0; return NodeStatus::FAILURE; } } }
  void halt() override { ControlNode::halt(); idx = 0; retries = 0; } };
class RoundRobin : public ControlNode { public: using ControlNode::ControlNode; size_t cur = 0, fails = 0;
  static PortsList providedPorts() { return {}; }
  NodeStatus tick() override { setStatus(NodeStatus::RUNNING); const size_t n = childrenCount();
    while (fails < n) { auto s = children_nodes_[cur]->executeTick();
      if (s == NodeStatus::RUNNING) return s;
      haltChild(cur); cur = (cur + 1) % n;
      if (s == NodeStatus::SUCCESS) { fails = 0; return s; } fails++; }
    fails = 0; return NodeStatus::FAILURE; }
  void halt() override { ControlNode::halt(); fails = 0; } };

int run(const std::string& name, std::function<void(int)> inject, int max_ticks, std::function<bool()> ok) {
  g = G{}; BehaviorTreeFactory f;
  f.registerNodeType<WaitForTask>("WaitForTask"); f.registerNodeType<AcceptTask>("AcceptTask");
  f.registerBuilder<Gate>("SafetyGate", [](const std::string& n, const NodeConfig& c) { return std::make_unique<Gate>(n, c, &g.estop); });
  f.registerBuilder<Gate>("TrafficGate", [](const std::string& n, const NodeConfig& c) { return std::make_unique<Gate>(n, c, &g.hold); });
  f.registerNodeType<NavigateTo>("NavigateTo"); f.registerNodeType<DockToStation>("DockToStation"); f.registerNodeType<ChargeUntil>("ChargeUntil");
  f.registerNodeType<IsBatteryLow>("IsBatteryLow"); f.registerNodeType<HasPendingTask>("HasPendingTask"); f.registerNodeType<IsMarkerVisible>("IsMarkerVisible");
  f.registerNodeType<IsDeadlineAtRisk>("IsDeadlineAtRisk"); f.registerNodeType<SetSpeedLimit>("SetSpeedLimit"); f.registerNodeType<ClearCostmaps>("ClearCostmaps");
  f.registerNodeType<BackUp>("BackUp"); f.registerNodeType<Spin>("Spin"); f.registerNodeType<SearchMarker>("SearchMarker");
  f.registerNodeType<BackToStaging>("BackToStaging"); f.registerNodeType<VirtualLoad>("VirtualLoad"); f.registerNodeType<VirtualUnload>("VirtualUnload");
  f.registerNodeType<PublishTaskEvent>("PublishTaskEvent"); f.registerNodeType<LogTaskRecord>("LogTaskRecord");
  f.registerNodeType<RequestReassignment>("RequestReassignment"); f.registerNodeType<RecoveryNode>("RecoveryNode"); f.registerNodeType<RoundRobin>("RoundRobin");
  auto tree = f.createTreeFromFile("/w/mission.xml");
  auto bb = tree.rootBlackboard(); bb->set("task_phase", 0); bb->set("fail_reason", std::string("NONE"));
  bb->set("home_pose", std::string("home")); bb->set("charger_staging", std::string("charger_staging")); bb->set("charger_dock", std::string("charger"));
  NodeStatus s = NodeStatus::IDLE; bool root_failed = false;
  try { for (int k = 0; k < max_ticks; k++) { inject(k); s = tree.tickOnce(); if (s == NodeStatus::FAILURE) root_failed = true;
          std::this_thread::sleep_for(std::chrono::milliseconds(2)); } }
  catch (std::exception& e) { std::cout << name << ": EXCEPTION " << e.what() << "\n"; return 1; }
  bool pass = ok() && !root_failed && s == NodeStatus::RUNNING;
  std::cout << (pass ? "PASS " : "FAIL ") << name << " | root=" << toStr(s) << " task_phase=" << bb->get<int>("task_phase") << " | ";
  for (auto& e : g.log) std::cout << e << " "; std::cout << "\n"; return pass ? 0 : 1; }

int main() {
  int bad = 0;
  bad += run("A normal", [](int k) { if (k == 0) g.tasks = 1; }, 200,
             [] { return count("EVENT:COMPLETED") == 1 && count("NAV_OK:home") == 1 && count("DOCK:pickup") == 1 && count("DOCK:dropoff") == 1; });
  bad += run("B dock fails x3", [](int k) { if (k == 0) { g.tasks = 1; g.dock_fail_pickup = true; } }, 300,
             [] { return count("DOCK:pickup") == 3 && count("BACK_TO_STAGING") == 3 && count("EVENT:FAILED") == 1 && count("REASSIGN") == 1
                        && count("REASON:LATERAL_DEVIATION") == 1 && count("NAV_OK:home") == 1 && count("EVENT:COMPLETED") == 0; });
  bad += run("C estop during dropoff nav", [](int k) { if (k == 0) g.tasks = 1;
               static int t0 = -1; if (k == 0) t0 = -1;
               if (t0 < 0 && count("NAV:dropoff_staging") == 1) t0 = k; g.estop = (t0 >= 0 && k >= t0 + 1 && k < t0 + 5); }, 300,
             [] { return count("NAV_CANCEL:dropoff_staging") == 1 && count("NAV:dropoff_staging") == 2 && count("DOCK:pickup") == 1
                        && count("LOAD") == 1 && count("ACCEPT") == 1 && count("EVENT:COMPLETED") == 1; });
  bad += run("D nav always fails", [](int k) { if (k == 0) { g.tasks = 1; g.nav_fail = true; } }, 2500,
             [] { return count("REC:clear") >= 1 && count("REC:spin") >= 1 && count("EVENT:FAILED") == 1 && count("REASON:NAV_FAILED") == 1 && count("DOCK:pickup") == 0; });
  bad += run("E battery preempts mid-task", [](int k) { if (k == 0) g.tasks = 1;
               static bool done = false; if (k == 0) done = false;
               if (!done && count("NAV:dropoff_staging") == 1) { g.battery = 15; done = true; } }, 400,
             [] { return count("NAV_CANCEL:dropoff_staging") == 1 && count("CHARGED") == 1 && count("DOCK:charger") == 1
                        && count("DOCK:pickup") == 1 && count("LOAD") == 1 && count("EVENT:COMPLETED") == 1; });
  bad += run("F traffic hold during nav", [](int k) { if (k == 0) g.tasks = 1; g.hold = (k >= 2 && k < 6); }, 200,
             [] { return count("NAV_CANCEL:pickup_staging") == 1 && count("EVENT:COMPLETED") == 1; });
  std::cout << (bad ? "SOME FAILED" : "ALL PASS") << "\n"; return bad; }
