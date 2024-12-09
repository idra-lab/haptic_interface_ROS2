#ifndef __SYSTEM_INTERFACE_HPP__
#define __SYSTEM_INTERFACE_HPP__
// this is the class that controls the haptic interface using its API,
// you can redefine it to adapt the library to your device
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "raptor_api_interfaces/msg/in_virtuose_force.hpp"
#include "raptor_api_interfaces/msg/in_virtuose_pose.hpp"
#include "raptor_api_interfaces/msg/in_virtuose_speed.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_force.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_physical_pose.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_pose.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_speed.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_status.hpp"
#include "raptor_api_interfaces/srv/virtuose_impedance.hpp"
#include "raptor_api_interfaces/srv/virtuose_reset.hpp"

inline double filter_force(double alpha, double v, double v_old) {
  return alpha * v_old + (1 - alpha) * v;
}
class SystemInterface : public rclcpp::Node {
 public:
  explicit SystemInterface(
      const Eigen::Quaterniond& q_haptic_base_to_robot_base,
      const std::string& name = "system_interface",
      const std::string& namespace_ = "",
      const rclcpp::NodeOptions& options =
          rclcpp::NodeOptions()
              .allow_undeclared_parameters(true)
              .automatically_declare_parameters_from_overrides(true))
      : Node(name, namespace_, options),
        q_haptic_base_to_robot_base_(q_haptic_base_to_robot_base) {
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing publishers");
    out_virtuose_status_ = this->create_subscription<
        raptor_api_interfaces::msg::OutVirtuoseStatus>(
        "out_virtuose_status", 1,
        std::bind(&SystemInterface::out_virtuose_statusCB, this,
                  std::placeholders::_1));
    _out_virtuose_pose_ =
        this->create_subscription<raptor_api_interfaces::msg::OutVirtuosePose>(
            "out_virtuose_pose", 1,
            std::bind(&SystemInterface::out_virtuose_pose_CB, this,
                      std::placeholders::_1));

    _in_virtuose_force =
        this->create_publisher<raptor_api_interfaces::msg::InVirtuoseForce>(
            "in_virtuose_force", 1);
    impedance_client_ =
        this->create_client<raptor_api_interfaces::srv::VirtuoseImpedance>(
            "virtuose_impedance");

    // Constructor body if needed
  }
  void create_connection() {
    auto imp = std::make_shared<
        raptor_api_interfaces::srv::VirtuoseImpedance::Request>();
    imp->channel = "SimpleChannelUDP";
    imp->ff_device_ip_address = "192.168.100.53";
    imp->ff_device_param_file = "/etc/Haption/Connector/desktop_6D_n65.param";
    imp->local_ip_address = "192.168.100.50";

    imp->base_frame.translation.x = 0.0;
    imp->base_frame.translation.y = 0.0;
    imp->base_frame.translation.z = 0.0;

    imp->base_frame.rotation.x = q_haptic_base_to_robot_base_.x();
    imp->base_frame.rotation.y = q_haptic_base_to_robot_base_.y();
    imp->base_frame.rotation.z = q_haptic_base_to_robot_base_.z();
    imp->base_frame.rotation.w = q_haptic_base_to_robot_base_.w();

    while (!impedance_client_->wait_for_service(
        std::literals::chrono_literals::operator""s(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "service not available, waiting again...");
    }

    auto result = impedance_client_->async_send_request(imp);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      // Store client_ ID given by virtuose_node
      client__id_ = result.get()->client_id;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Our client_ ID is: %d",
                  client__id_);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Failed to call service impedance");
      return;
    }

    if (client__id_ == 0) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Failed to call service impedance, client__id_ is zero!");
      return;
    }

    ctr_ = 0;

    // Perform impedance loop at 1000 Hz
    impedanceThread_ = this->create_wall_timer(
        std::literals::chrono_literals::operator""s(1),
        std::bind(&SystemInterface::impedance_thread, this));
    RCLCPP_INFO(this->get_logger(),
                "\033[0;32mImpedance thread started\033[0m");
  }

  void apply_target_wrench(geometry_msgs::msg::WrenchStamped target_wrench) {
    force_.header.stamp.nanosec = target_wrench.header.stamp.nanosec;
    force_.header.stamp.sec = target_wrench.header.stamp.sec;
    force_.client_id = client__id_;
    // filter noise
    force_.virtuose_force.force.x =
        scale_ * filter_force(alpha_, target_wrench.wrench.force.x,
                              old_force_.virtuose_force.force.x);
    force_.virtuose_force.force.y =
        scale_ * filter_force(alpha_, target_wrench.wrench.force.y,
                              old_force_.virtuose_force.force.y);
    force_.virtuose_force.force.z =
        scale_ * filter_force(alpha_, target_wrench.wrench.force.z,
                              old_force_.virtuose_force.force.z);

    // torque omitted for control simplicity
    force_.virtuose_force.torque.x = 0.0;
    force_.virtuose_force.torque.y = 0.0;
    force_.virtuose_force.torque.z = 0.0;

    // SAFE ZONE FORCE

    force.virtuose_force.force.x =
        std::clamp(force.virtuose_force.force.x, -max_force_, max_force_);
    force.virtuose_force.force.y =
        std::clamp(force.virtuose_force.force.y, -max_force_, max_force_);
    force.virtuose_force.force.z =
        std::clamp(force.virtuose_force.force.z, -max_force_, max_force_);

    // updating old force
    old_force_.virtuose_force.force.x = force.virtuose_force.force.x;
    old_force_.virtuose_force.force.y = force.virtuose_force.force.y;
    old_force_.virtuose_force.force.z = force.virtuose_force.force.z;
    old_force_.virtuose_force.torque.x = force.virtuose_force.torque.x;
    old_force_.virtuose_force.torque.y = force.virtuose_force.torque.y;
    old_force_.virtuose_force.torque.z = force.virtuose_force.torque.z;

    _in_virtuose_force->publish(force);
    ctr_++;
  }

 private:
  raptor_api_interfaces::msg::InVirtuoseForce force_;
  raptor_api_interfaces::msg::InVirtuoseForce old_force_;
  int client__id_, ctr_ = 0;
  const double alpha_ = 0.0;
  const double scale_ = 0.3;
  const double max_force_ = 6.0;
  Eigen::Quaterniond q_haptic_base_to_robot_base_;

  rclcpp::Publisher<raptor_api_interfaces::msg::InVirtuoseForce>::SharedPtr
      _in_virtuose_force;
  rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuoseStatus>::SharedPtr
      out_virtuose_status_;
  rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuosePose>::SharedPtr
      _out_virtuose_pose_;
  rclcpp::Client<raptor_api_interfaces::srv::VirtuoseImpedance>::SharedPtr
      impedance_client_;
  void out_virtuose_pose_CB(
      const raptor_api_interfaces::msg::OutVirtuosePose::SharedPtr msg) {}
  void out_virtuose_statusCB(
      const raptor_api_interfaces::msg::OutVirtuoseStatus::SharedPtr msg) {}
  // thread
  rclcpp::TimerBase::SharedPtr impedanceThread_;
  void impedance_thread(){};
}

#endif  // __SYSTEM_INTERFACE_HPP__
