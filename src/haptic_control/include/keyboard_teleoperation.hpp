#ifndef KEYBOARD_TELEOPERATION_HPP
#define KEYBOARD_TELEOPERATION_HPP
/*
Sample example of teleoperation using haptic device and force feedback
@Author: Davide Nardi
*/

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
#include "conic_cbf/conic_cbf.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "mesh_virtual_fixtures/vf_enforcer.hpp"

#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <map>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace Eigen;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class KeyboardControl : public rclcpp::Node {
 public:
  KeyboardControl(
      const std::string &name = "keyboard_control",
      const rclcpp::NodeOptions &options =
          rclcpp::NodeOptions()
              .allow_undeclared_parameters(true)
              .automatically_declare_parameters_from_overrides(true));
  void projectTargetOnSphere(Eigen::Vector3d &target_position_vec,
                             double safety_sphere_radius_);
  void readInput();
  void init_vf_enforcer();
  void init_control();
  void controlThread();

 private:
  // Map for movement keys
  // std::map<char, std::vector<float>> moveBindings{
  //     {'w', {1, 0, 0}},  {'s', {-1, 0, 0}}, {'a', {0, 1, 0}},
  //     {'d', {0, -1, 0}}, {'q', {0, 0, 1}},  {'e', {0, 0, -1}}
  //     {''
  //   };

  // Reminder message
  const char *msg = R"(
Reading from the keyboard!
---------------------------
Moving around:
 q    w    e
 a    s    d

Press Ctrl-C to quit.
)";

  // Init variables
  char key = ' ';
  const double motion_scale_l = 0.01;
  const double motion_scale_a = 0.01;
  // For non-blocking keyboard inputs
  int getch(void) {
    int ch = -1;  // Default to -1 if no input is available
    struct termios oldt, newt;
    int oldf;

    // Store old terminal settings and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Set terminal to raw mode
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Set stdin to non-blocking mode
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    // Attempt to read a character
    ch = getchar();

    // Restore old terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);  // Restore old file status flags

    return ch;  // Will return -1 if no input was available
  }

  // ROS2 subscribtions
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_pos_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      current_target_pos_publisher_;
  // ROS2 timers
  rclcpp::TimerBase::SharedPtr pose_update_timer_;
  rclcpp::TimerBase::SharedPtr controlThread_;
  // ROS2 tf2 transform listener and buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // ROS2 params callbacks
  geometry_msgs::msg::PoseStamped target_pose_, current_pose_;
  geometry_msgs::msg::TransformStamped target_pose_tf_;
  geometry_msgs::msg::PoseStamped old_pose_;
  geometry_msgs::msg::PoseStamped ee_starting_pose;
  geometry_msgs::msg::PoseStamped ee_current_pose_;
  geometry_msgs::msg::PoseStamped haptic_starting_position_;

  std::string tool_link_name_;
  std::string base_link_name_;

  bool received_ee_pose_;
  bool use_vf_ = false;
  Eigen::Vector3d euler_angles_;
  Eigen::Quaterniond q_new, q_old, q_init;
  Eigen::Vector3d thetas;
  std::array<double, 7> cur_pose_;
  std::array<double, 3> bounding_box_center_;
  std::shared_ptr<VFEnforcer> vf_enforcer_;

  // current haptic positions
  Eigen::Vector3d x_new_;
  // current and old haptic displacements
  Eigen::Vector3d x_tilde_new_;
};

#endif  // KEYBOARD_TELEOPERATION_HPP