// Checks BT.CPP v4 semantics used by the brief's MissionLoop (throwaway container only).
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>
using namespace BT;
static bool estop=false; static int halted=0; static int ticks_task=0;
class SafetyGate: public StatefulActionNode{ public:
  SafetyGate(const std::string&n,const NodeConfig&c):StatefulActionNode(n,c){}
  static PortsList providedPorts(){return {};}
  NodeStatus onStart() override { return estop?NodeStatus::RUNNING:NodeStatus::SUCCESS; }
  NodeStatus onRunning() override { return estop?NodeStatus::RUNNING:NodeStatus::SUCCESS; }
  void onHalted() override {} };
class LongTask: public StatefulActionNode{ public:
  LongTask(const std::string&n,const NodeConfig&c):StatefulActionNode(n,c){}
  static PortsList providedPorts(){return {};}
  NodeStatus onStart() override { ticks_task=0; return NodeStatus::RUNNING; }
  NodeStatus onRunning() override { return (++ticks_task>=5)?NodeStatus::FAILURE:NodeStatus::RUNNING; }
  void onHalted() override { halted++; } };
int main(){
  BehaviorTreeFactory f; f.registerNodeType<SafetyGate>("SafetyGate"); f.registerNodeType<LongTask>("LongTask");
  const char* xml = R"(<root BTCPP_format="4"><BehaviorTree ID="Main">
   <KeepRunningUntilFailure><ForceSuccess><ReactiveSequence>
     <SafetyGate/>
     <Fallback> <LongTask/> </Fallback>
   </ReactiveSequence></ForceSuccess></KeepRunningUntilFailure></BehaviorTree></root>)";
  auto tree=f.createTreeFromText(xml);
  try{
    for(int k=0;k<14;k++){ estop=(k>=2&&k<4); auto s=tree.tickOnce();
      std::cout<<"tick "<<k<<" estop="<<estop<<" root="<<toStr(s)<<" halted="<<halted<<" ticks_task="<<ticks_task<<"\n"; }
  }catch(std::exception&e){ std::cout<<"EXCEPTION: "<<e.what()<<"\n"; }
  // Script resume check: does a skipped phase's Script regress task_phase?
  const char* xml2 = R"(<root BTCPP_format="4"><BehaviorTree ID="M"><Sequence>
     <AlwaysSuccess _skipIf="task_phase >= 2"/> <Script code="task_phase:=2"/>
     <AlwaysSuccess _skipIf="task_phase >= 3"/> <Script code="task_phase:=3"/>
   </Sequence></BehaviorTree></root>)";
  auto t2=f.createTreeFromText(xml2); t2.rootBlackboard()->set("task_phase",5); t2.tickOnce();
  std::cout<<"flat Script form: task_phase after resume from 5 = "<<t2.rootBlackboard()->get<int>("task_phase")<<"\n";
  const char* xml3 = R"(<root BTCPP_format="4"><BehaviorTree ID="M"><Sequence>
     <Sequence _skipIf="task_phase >= 2"><AlwaysSuccess/> <Script code="task_phase:=2"/></Sequence>
     <Sequence _skipIf="task_phase >= 3"><AlwaysSuccess/> <Script code="task_phase:=3"/></Sequence>
   </Sequence></BehaviorTree></root>)";
  auto t3=f.createTreeFromText(xml3); t3.rootBlackboard()->set("task_phase",5); auto s3=t3.tickOnce();
  std::cout<<"wrapped form: task_phase after resume from 5 = "<<t3.rootBlackboard()->get<int>("task_phase")<<" status="<<toStr(s3)<<"\n";
  return 0; }
