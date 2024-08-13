# include "vf_test.hpp"

int main(int argc, char **argv) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting VF Haptic Control node"); 
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VirtualFixtureTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}