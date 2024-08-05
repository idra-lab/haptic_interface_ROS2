# include "virtual_fixture_control.hpp"

int main(int argc, char **argv) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Haptic Control node");

  rclcpp::init(argc, argv);
  auto node = std::make_shared<VirtualFixtureControl>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling impedance service:");

  node->call_impedance_service();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}