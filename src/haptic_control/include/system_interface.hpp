#ifndef __SYSTEM_INTERFACE_HPP__
#define __SYSTEM_INTERFACE_HPP__
// this is the class that controls the haptic interface using its API,
// you can redefine it to adapt the library to your device.
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
#include "raptor_api_interfaces/srv/virtuose_calibrate.hpp"
#include "raptor_api_interfaces/srv/virtuose_impedance.hpp"
#include "raptor_api_interfaces/srv/virtuose_reset.hpp"

inline double filter_force(double alpha, double v, double v_old) {
  return alpha * v_old + (1 - alpha) * v;
}
using namespace std::chrono_literals;
class SystemInterface : public rclcpp::Node {
 public:
  explicit SystemInterface(
      const std::string channel, const std::string local_ip_address,
      const std::string device_ip_address, const std::string device_param_file,
      const std::vector<double> q_haptic_base_to_robot_base,
      const double force_scale = 1.0, const double force_max = 6.0,
      const double smooth_factor = 0.5,
      const std::string& name = "system_interface",
      const std::string& namespace_ = "",
      const rclcpp::NodeOptions& options =
          rclcpp::NodeOptions()
              .allow_undeclared_parameters(true)
              .automatically_declare_parameters_from_overrides(true))
      : Node(name, namespace_, options) {
    pose_subscriber_ =
        this->create_subscription<raptor_api_interfaces::msg::OutVirtuosePose>(
            "out_virtuose_pose", 1,
            std::bind(&SystemInterface::store_pose, this,
                      std::placeholders::_1));
    wrench_publisher_ =
        this->create_publisher<raptor_api_interfaces::msg::InVirtuoseForce>(
            "in_virtuose_force", 1);
    calibration_client_ =
        this->create_client<raptor_api_interfaces::srv::VirtuoseCalibrate>(
            "virtuose_calibrate");
    impedance_client_ =
        this->create_client<raptor_api_interfaces::srv::VirtuoseImpedance>(
            "virtuose_impedance");
    channel_ = channel;
    local_ip_address_ = local_ip_address;
    device_ip_address_ = device_ip_address;
    device_param_file_ = device_param_file;
    q_haptic_base_to_robot_base_ = q_haptic_base_to_robot_base;
    f_scale_ = force_scale;
    f_max_ = force_max;
    alpha_ = smooth_factor;
  }
  void create_connection() {
    auto cal = std::make_shared<
        raptor_api_interfaces::srv::VirtuoseCalibrate::Request>();
    auto imp = std::make_shared<
        raptor_api_interfaces::srv::VirtuoseImpedance::Request>();
    imp->channel = cal->channel = channel_;
    imp->ff_device_ip_address = cal->ff_device_ip_address = device_ip_address_;
    imp->local_ip_address = cal->local_ip_address = local_ip_address_;
    imp->ff_device_param_file = cal->ff_device_param_file = device_param_file_;

    imp->base_frame.translation.x = 0.0;
    imp->base_frame.translation.y = 0.0;
    imp->base_frame.translation.z = 0.0;

    imp->base_frame.rotation.x = q_haptic_base_to_robot_base_[0];
    imp->base_frame.rotation.y = q_haptic_base_to_robot_base_[1];
    imp->base_frame.rotation.z = q_haptic_base_to_robot_base_[2];
    imp->base_frame.rotation.w = q_haptic_base_to_robot_base_[3];

    while (!calibration_client_->wait_for_service(
        std::literals::chrono_literals::operator""s(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Interrupted while waiting for the calibration service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "Calibration service not available, waiting again...");
    }
    auto calibration_result = calibration_client_->async_send_request(cal);
    bool success = false;
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           calibration_result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      success = calibration_result.get()->success;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration result: %d",
                  success);
      // return;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Failed to call service calibration");
      rclcpp::shutdown();
    }

    if (!success) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to calibrate!");
      rclcpp::shutdown();
    }

    while (!impedance_client_->wait_for_service(
        std::literals::chrono_literals::operator""s(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Interrupted while waiting for the impedance service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "Impedance service not available, waiting again...");
    }

    auto impedance_result = impedance_client_->async_send_request(imp);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           impedance_result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      // Store client_ ID given by virtuose_node
      client__id_ = impedance_result.get()->client_id;
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
  }
  void start_force_feedback() {
    set_target_wrench_timer_ = this->create_wall_timer(
        1ms, std::bind(&SystemInterface::apply_target_wrench, this));
  }

  void update_target_wrench(
      const geometry_msgs::msg::WrenchStamped target_wrench) {
    target_wrench_ = target_wrench;
  }

  void apply_target_wrench() {
    if (!received_haptic_pose_) {
      return;
    }
    raptor_api_interfaces::msg::InVirtuoseForce force_;
    force_.client_id = client__id_;
    // filter noise
    force_.virtuose_force.force.x =
        f_scale_ * filter_force(alpha_, target_wrench_.wrench.force.x,
                                old_force_.virtuose_force.force.x);
    force_.virtuose_force.force.y =
        f_scale_ * filter_force(alpha_, target_wrench_.wrench.force.y,
                                old_force_.virtuose_force.force.y);
    force_.virtuose_force.force.z =
        f_scale_ * filter_force(alpha_, target_wrench_.wrench.force.z,
                                old_force_.virtuose_force.force.z);

    // torque omitted for control simplicity
    force_.virtuose_force.torque.x = 0.0;
    force_.virtuose_force.torque.y = 0.0;
    force_.virtuose_force.torque.z = 0.0;

    // SAFE ZONE FORCE
    force_.virtuose_force.force.x =
        std::clamp(force_.virtuose_force.force.x, -f_max_, f_max_);
    force_.virtuose_force.force.y =
        std::clamp(force_.virtuose_force.force.y, -f_max_, f_max_);
    force_.virtuose_force.force.z =
        std::clamp(force_.virtuose_force.force.z, -f_max_, f_max_);

    // updating old force
    old_force_ = force_;

    force_.header.stamp = get_clock()->now();
    wrench_publisher_->publish(force_);
  }

  void store_pose(
      const raptor_api_interfaces::msg::OutVirtuosePose::SharedPtr msg) {
    if (!received_haptic_pose_) {
      // Store the first pose received
      received_haptic_pose_ = true;
      haptic_starting_pose_.pose.position.x = msg->virtuose_pose.translation.x;
      haptic_starting_pose_.pose.position.y = msg->virtuose_pose.translation.y;
      haptic_starting_pose_.pose.position.z = msg->virtuose_pose.translation.z;
      haptic_starting_pose_.pose.orientation.x = msg->virtuose_pose.rotation.x;
      haptic_starting_pose_.pose.orientation.y = msg->virtuose_pose.rotation.y;
      haptic_starting_pose_.pose.orientation.z = msg->virtuose_pose.rotation.z;
      haptic_starting_pose_.pose.orientation.w = msg->virtuose_pose.rotation.w;
    }
    haptic_current_pose_.pose.position.x = msg->virtuose_pose.translation.x;
    haptic_current_pose_.pose.position.y = msg->virtuose_pose.translation.y;
    haptic_current_pose_.pose.position.z = msg->virtuose_pose.translation.z;
    haptic_current_pose_.pose.orientation.x = msg->virtuose_pose.rotation.x;
    haptic_current_pose_.pose.orientation.y = msg->virtuose_pose.rotation.y;
    haptic_current_pose_.pose.orientation.z = msg->virtuose_pose.rotation.z;
    haptic_current_pose_.pose.orientation.w = msg->virtuose_pose.rotation.w;
  }

  geometry_msgs::msg::PoseStamped haptic_starting_pose_, haptic_current_pose_;
  bool received_haptic_pose_ = false;
  geometry_msgs::msg::WrenchStamped target_wrench_;

 private:
  raptor_api_interfaces::msg::InVirtuoseForce old_force_;
  int client__id_ = 0;
  double f_scale_, f_max_, alpha_;
  std::string channel_, local_ip_address_, device_ip_address_,
      device_param_file_;
  std::vector<double> q_haptic_base_to_robot_base_;

  rclcpp::Publisher<raptor_api_interfaces::msg::InVirtuoseForce>::SharedPtr
      wrench_publisher_;

  rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuosePose>::SharedPtr
      pose_subscriber_;
  rclcpp::Client<raptor_api_interfaces::srv::VirtuoseImpedance>::SharedPtr
      impedance_client_;
  rclcpp::Client<raptor_api_interfaces::srv::VirtuoseCalibrate>::SharedPtr
      calibration_client_;
  rclcpp::TimerBase::SharedPtr set_target_wrench_timer_;
};

#endif  // __SYSTEM_INTERFACE_HPP__
