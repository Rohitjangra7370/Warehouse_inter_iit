#include "warehouse_rover_mission_control/mission_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<warehouse_rover_mission_control::MissionController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
