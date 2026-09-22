// docking_server_node 실행 파일.
#include <memory>

#include "amr_behavior/docking/docking_server_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_behavior::docking::DockingServerNode>());
  rclcpp::shutdown();
  return 0;
}
