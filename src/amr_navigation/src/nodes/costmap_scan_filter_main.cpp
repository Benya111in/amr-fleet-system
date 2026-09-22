#include <memory>

#include "amr_navigation/costmap_scan_filter_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_navigation::CostmapScanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
