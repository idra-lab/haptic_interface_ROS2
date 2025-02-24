/*
Teleoperation node using haptic device and force feedback which implements the
mesh virtual fixtures explained in the paper
https://ieeexplore.ieee.org/document/9341590/
@Author: Davide Nardi
*/
#ifndef __HAPTIC_CONTROL_BASE__
#define __HAPTIC_CONTROL_BASE__
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <limits>
#include <memory>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "circular_buffer.hpp"
#include "mesh_virtual_fixtures/vf_enforcer.hpp"
#include "system_interface.hpp"

class HapticControlBase : public rclcpp::Node {
 public:
  HapticControlBase(
      const std::string &name = "haptic_control",
      const std::string &namespace_ = "",
      const rclcpp::NodeOptions &options =
          rclcpp::NodeOptions()
              .allow_undeclared_parameters(true)
              .automatically_declare_parameters_from_overrides(true));
  // safe sphere virtual fixture
  void enable_safety_sphere_CB(const rclcpp::Parameter &p);
  void set_safety_sphere_radius_CB(const rclcpp::Parameter &p);
  // safe box virtual fixture
  void enable_safety_box_CB(const rclcpp::Parameter &p);
  void set_safety_box_width_CB(const rclcpp::Parameter &p);
  void set_safety_box_length_CB(const rclcpp::Parameter &p);
  void set_safety_box_height_CB(const rclcpp::Parameter &p);
  void update_current_ee_pos();
  void store_wrench(const geometry_msgs::msg::WrenchStamped target_wrench);
  Eigen::Vector3d compute_position_error();
  Eigen::Quaterniond compute_orientation_error();
  void initialize_haptic_control();
  void control_thread();
  void get_ee_trans(geometry_msgs::msg::TransformStamped &trans);
  void init_vf_enforcer();
  void project_target_on_sphere(Eigen::Vector3d &target_position_vec,
                                double safety_sphere_radius_);
  geometry_msgs::msg::TransformStamped target_pose_tf_;
  geometry_msgs::msg::PoseStamped target_pose_, target_pose_vf_, current_pose_,
      old_target_pose_vf_;
  std::string base_link_name_;
  bool use_fixtures_ = true;
  std::shared_ptr<SystemInterface> haptic_device_;

 private:
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      ft_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_frame_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      current_frame_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      desired_frame_pub_;

  // ROS2 timers
  rclcpp::TimerBase::SharedPtr pose_update_timer_;
  // ROS2 tf2 transform listener and buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // ROS2 params callbacks
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_enable_safety_sphere_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_safety_sphere_radius_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_enable_safety_box_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_safety_box_width_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_safety_box_length_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_safety_box_height_;

  geometry_msgs::msg::WrenchStamped current_wrench_;
  geometry_msgs::msg::PoseStamped old_pose_;
  geometry_msgs::msg::PoseStamped ee_starting_position;
  geometry_msgs::msg::PoseStamped ee_current_pose_;
  geometry_msgs::msg::PoseStamped haptic_starting_pose_;
  Eigen::Quaterniond q_haptic_base_to_robot_base_, qEEStart;
  double safety_sphere_radius_, safety_box_width_, safety_box_length_,
      safety_box_height_;
  double max_force_;
  double force_scale_;
  std::string tool_link_name_;
  std::string ft_link_name_;
  std::string target_frame_topic_name_, ft_feedback_topic_name_;
  bool received_haptic_pose_;
  bool received_ee_pose_;
  bool enable_safety_sphere_, enable_safety_box_;
  // delay simulation
  double delay_;
  int delay_loop_haptic_, delay_loop_ft_;
  double haptic_control_rate_, ft_sensor_rate_;
  // time consistency check
  rclcpp::Time last_robot_pose_update_time_;
  //   std::vector<geometry_msgs::msg::WrenchStamped> wrench_history_buffer_;
  // std::vector<geometry_msgs::msg::PoseStamped> target_pose_history_buffer_,
  // target_pose_vf_history_buffer_;
  CircularBuffer<geometry_msgs::msg::WrenchStamped> wrench_buffer_;
  CircularBuffer<geometry_msgs::msg::PoseStamped> target_pose_buffer_,
      target_pose_vf_buffer_;

  // Storage for virtuose_node status
  std::array<double, 7> cur_pose_;
  std::array<double, 3> bounding_box_center_;
  uint32_t status_state_;
  uint32_t status_button_;
  // current and old hapitc positions
  Eigen::Vector3d x_new_, x_old_;
  // current and old haptic displacements
  Eigen::Vector3d x_tilde_old_, x_tilde_new_;

  std::shared_ptr<VFEnforcer> vf_enforcer_;
  // ros control_thread_
  rclcpp::TimerBase::SharedPtr control_thread_;
};

#endif  // __HAPTIC_CONTROL_BASE__