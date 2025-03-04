/*
Sample example of teleoperation using haptic device and force feedback
@Author: Davide Nardi
*/
#ifndef SAMPLE_TELEOPERATION_HPP
#define SAMPLE_TELEOPERATION_HPP

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <limits>
#include <memory>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
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

using namespace Eigen;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class HapticControl : public rclcpp::Node {
 public:
  HapticControl(const std::string &name = "haptic_control",
                const std::string &namespace_ = "",
                const rclcpp::NodeOptions &options =
                    rclcpp::NodeOptions()
                        .allow_undeclared_parameters(true)
                        .automatically_declare_parameters_from_overrides(true));
  void enable_safety_sphere_CB(const rclcpp::Parameter &p);
  void set_safety_sphere_radius_CB(const rclcpp::Parameter &p);
  void enable_safety_box_CB(const rclcpp::Parameter &p);
  void set_safety_box_width_CB(const rclcpp::Parameter &p);
  void set_safety_box_length_CB(const rclcpp::Parameter &p);
  void set_safety_box_height_CB(const rclcpp::Parameter &p);
  void update_current_ee_pos();
  void SetWrenchCB(const geometry_msgs::msg::WrenchStamped target_wrench);
  void out_virtuose_pose_CB(
      const raptor_api_interfaces::msg::OutVirtuosePose::SharedPtr msg);
  void out_virtuose_statusCB(
      const raptor_api_interfaces::msg::OutVirtuoseStatus::SharedPtr msg);
  void callImpedanceService();
  void impedanceThread();
  void projectTargetOnSphere(Eigen::Vector3d &target_position_vec,
                             double safety_sphere_radius_);

 private:
  // ROS2 subscribtions
  rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuoseStatus>::SharedPtr
      out_virtuose_status_;
  rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuosePose>::SharedPtr
      _out_virtuose_pose_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      ft_subscriber_;
  // ROS2 publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_pos_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      current_target_pos_publisher_;
  rclcpp::Publisher<raptor_api_interfaces::msg::InVirtuoseForce>::SharedPtr
      _in_virtuose_force;
  // ROS2 service client_s
  rclcpp::Client<raptor_api_interfaces::srv::VirtuoseImpedance>::SharedPtr
      impedance_client_;
  rclcpp::Client<raptor_api_interfaces::srv::VirtuoseCalibrate>::SharedPtr
      calibration_client_;
  // ROS2 timers
  rclcpp::TimerBase::SharedPtr pose_update_timer_;
  rclcpp::TimerBase::SharedPtr impedanceThread_;
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
  geometry_msgs::msg::PoseStamped target_pose_, current_pose_;
  geometry_msgs::msg::TransformStamped target_pose_tf_;
  geometry_msgs::msg::PoseStamped old_pose_;
  geometry_msgs::msg::PoseStamped ee_starting_position;
  geometry_msgs::msg::PoseStamped ee_current_pose_;
  geometry_msgs::msg::PoseStamped haptic_starting_position_;
  Eigen::Quaterniond q_haptic_base_to_robot_base_;
  raptor_api_interfaces::msg::InVirtuoseForce old_force_;
  double safety_sphere_radius_, safety_box_width_, safety_box_length_,
      safety_box_height_;
  double max_force_;
  double force_scale_;
  std::string tool_link_name_;
  std::string base_link_name_;
  std::string ft_link_name_;
  std::string ft_topic_name_;
  bool received_haptic_pose_;
  bool received_ee_pose_;
  bool enable_safety_sphere_, enable_safety_box_;
  // Storage for virtuose_node status
  int64_t status_date_sec_;
  uint32_t status_date_nanosec_;
  int client__id_;
  int ctr_;
  // Storage for virtuose pose
  int64_t pose_date_sec_;
  uint32_t pose_date_nanosec_;
  std::array<double, 7> cur_pose_;
  std::array<double, 3> bounding_box_center_;
  uint32_t status_state_;
  uint32_t status_button_;

  // current and old hapitc positions
  Eigen::Vector3d x_new_, x_old_;
  // current and old haptic displacements
  Eigen::Vector3d x_tilde_old_, x_tilde_new_;
};

#endif  // SAMPLE_TELEOPERATION_HPP