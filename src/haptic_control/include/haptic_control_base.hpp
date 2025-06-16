/*
Teleoperation node using haptic device and force feedback implementing the
mesh virtual fixtures described in:
https://ieeexplore.ieee.org/document/9341590/

@Author: Davide Nardi
*/

#ifndef __HAPTIC_CONTROL_BASE__
#define __HAPTIC_CONTROL_BASE__

// ROS2 core
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/rclcpp.hpp>

// ROS2 TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// ROS2 Messages
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

// Eigen for math
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// STL
#include <array>
#include <chrono>
#include <limits>
#include <memory>
#include <string>

// Project-specific headers
#include "conic_cbf/conic_cbf.hpp"
#include "mesh_virtual_fixtures/vf_enforcer.hpp"
#include "system_interface.hpp"
#include "utils/circular_buffer.hpp"
#include "utils/conversions.hpp"
#include "utils/geometry.hpp"
#include "utils/visualization.hpp"

class HapticControlBase : public rclcpp::Node {
 public:
  // Constructor
  HapticControlBase(
      const std::string &name = "haptic_control",
      const std::string &namespace_ = "",
      const rclcpp::NodeOptions &options =
          rclcpp::NodeOptions()
              .allow_undeclared_parameters(true)
              .automatically_declare_parameters_from_overrides(true));

  // Initialization
  void initialize_haptic_control();
  void init_vf_enforcer();

  // Core control logic
  void control_thread();
  void update_current_ee_pos();
  void get_ee_transform(geometry_msgs::msg::TransformStamped &trans);

  // Utility functions
  void store_wrench(const geometry_msgs::msg::WrenchStamped &target_wrench);
  Eigen::Vector3d compute_position_error();
  Eigen::Quaterniond compute_orientation_error();

  // Public pose and transform data
  geometry_msgs::msg::TransformStamped target_pose_tf_;
  geometry_msgs::msg::PoseStamped target_pose_, target_pose_vf_, current_pose_;
  geometry_msgs::msg::TransformStamped ee_pose_;

  // Haptic control device node
  std::shared_ptr<SystemInterface> haptic_device_;

  // Public config flags
  bool use_fixtures_ = true;
  bool use_ccbf_ = true;
  bool use_initial_conf_as_q_ref_ = false;

 private:
  // ROS publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_frame_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      current_frame_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      desired_frame_pub_;

  // ROS subscriptions
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      ft_subscriber_;
  // PoseStamped subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      ee_pose_subscriber_;

  // ROS timers
  rclcpp::TimerBase::SharedPtr pose_update_timer_;
  rclcpp::TimerBase::SharedPtr control_thread_;  // For control loop

  // TF2 listener and broadcaster
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Pose and wrench data
  geometry_msgs::msg::WrenchStamped current_wrench_;
  geometry_msgs::msg::PoseStamped old_pose_, ee_starting_position,
      haptic_starting_pose_;
  CircularBuffer<geometry_msgs::msg::WrenchStamped> wrench_buffer_;
  CircularBuffer<geometry_msgs::msg::PoseStamped> target_pose_buffer_,
      target_pose_vf_buffer_;

  // Quaternion and position references
  Eigen::Quaterniond q_haptic_base_to_robot_base_, qEEStart_;
  Eigen::Quaterniond q_new_, q_old_, q_ref_;
  Eigen::Vector3d x_new_, x_old_, thetas_;
  Eigen::Vector3d x_tilde_old_, x_tilde_new_;

  // Device status and bounding box
  std::array<double, 7> cur_pose_;
  uint32_t status_state_;
  uint32_t status_button_;

  // Parameters and configuration values
  std::string base_link_name_;
  std::string tool_link_name_;
  std::string ft_link_name_;
  std::string target_frame_topic_name_;
  std::string ft_feedback_topic_name_;
  double safety_sphere_radius_;
  Eigen::Vector3d safety_sphere_center_;
  double delay_;
  int delay_loop_haptic_, delay_loop_ft_;
  double haptic_control_rate_, ft_sensor_rate_;
  double tool_vis_radius_;
  bool received_haptic_pose_ = false;
  bool received_ee_pose_ = false;
  bool first_control_loop_ = true;
  bool enable_safety_sphere_ = false;

  // Time tracking
  rclcpp::Time last_robot_pose_update_time_;

  // Visualizer and virtual fixture handler
  std::shared_ptr<Visualizer> vis_;
  std::shared_ptr<VFEnforcer> vf_enforcer_;
};

#endif  // __HAPTIC_CONTROL_BASE__