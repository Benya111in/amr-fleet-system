// Action ClearCostmap: Nav2 costmap 전체 비우기 서비스 (nav2_msgs/srv/ClearEntireCostmap).
// service_name 으로 전역/지역 costmap 을 고른다 (기본 = 전역).
#ifndef AMR_BEHAVIOR__BT_NODES__CLEAR_COSTMAP_HPP_
#define AMR_BEHAVIOR__BT_NODES__CLEAR_COSTMAP_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/ros_service_node.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

namespace amr_behavior
{

class ClearCostmapService : public RosServiceNode<nav2_msgs::srv::ClearEntireCostmap>
{
public:
  ClearCostmapService(
    const std::string & name, const BT::NodeConfiguration & config,
    RosNodeParams params)
  : RosServiceNode(
      name, config, std::move(params), "global_costmap/clear_entirely_global_costmap") {}

  static BT::PortsList providedPorts() {return providedBasicPorts({});}

protected:
  bool setRequest(Request & /*request*/) override {return true;}
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__CLEAR_COSTMAP_HPP_
