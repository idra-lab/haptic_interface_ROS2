#include "keyboard_teleoperation.hpp"

using namespace Eigen;

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
/*
Sample example of teleoperation using haptic device and force feedback
@Author: Davide Nardi
*/
KeyboardControl::KeyboardControl(const std::string &name,
                                 const std::string &namespace_,
                                 const rclcpp::NodeOptions &options)
    : Node(name, namespace_, options) {
  // safety sphere around robot base link to prevent singularity
  this->tool_link_name_ = this->get_parameter("tool_link_name").as_string();
  this->base_link_name_ = this->get_parameter("base_link_name").as_string();

  target_pos_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/lbr/target_frame", 1);
  current_target_pos_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/target_frame_vf", 1);

  // Initializes the TF2 transform listener and buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // defines the rotation from the robot base frame to the haptic base frame

  x_new_ << 0.0, 0.0, 0.0;
  x_old_ << 0.0, 0.0, 0.0;
  x_tilde_new_ << 0.0, 0.0, 0.0;
  x_tilde_old_ << 0.0, 0.0, 0.0;
}
void KeyboardControl::init_vf_enforcer() {
  // Init virtual fixture enforcer
  vf_enforcer_ = std::make_shared<VFEnforcer>(
      this->shared_from_this(),
      Eigen::Vector3d(cur_pose_[0], cur_pose_[1], cur_pose_[2]),
      this->base_link_name_);
}

void KeyboardControl::readInput() {
  key = getch();

  if (moveBindings.count(key) == 1) {
    // Grab the direction data
    cur_pose_[0] += motion_scale * moveBindings[key][0];
    cur_pose_[1] += motion_scale * moveBindings[key][1];
    cur_pose_[2] += motion_scale * moveBindings[key][2];
  }
  x_new_ << cur_pose_[0], cur_pose_[1], cur_pose_[2];
  x_old_ = x_new_;
  auto delta_x = vf_enforcer_->enforce_vf(x_new_);
  x_tilde_new_ += delta_x;
}

void KeyboardControl::startLoop() {
  received_ee_pose_ = false;
  while (!received_ee_pose_) {
    try {
      auto trans = tf_buffer_->lookupTransform(base_link_name_, tool_link_name_,
                                               tf2::TimePointZero);
      ee_starting_pose.pose.position.x = cur_pose_[0] =
          trans.transform.translation.x;
      ee_starting_pose.pose.position.y = cur_pose_[1] =
          trans.transform.translation.y;
      ee_starting_pose.pose.position.z = cur_pose_[2] =
          trans.transform.translation.z;
      ee_starting_pose.pose.orientation.x = cur_pose_[3] =
          trans.transform.rotation.x;
      ee_starting_pose.pose.orientation.y = cur_pose_[4] =
          trans.transform.rotation.y;
      ee_starting_pose.pose.orientation.z = cur_pose_[5] =
          trans.transform.rotation.z;
      ee_starting_pose.pose.orientation.w = cur_pose_[6] =
          trans.transform.rotation.w;
      received_ee_pose_ = true;
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "End Effector pose not available: %s", ex.what());
    }
    x_tilde_old_ = x_tilde_new_ = x_new_ = x_old_ =
        Eigen::Vector3d(cur_pose_[0], cur_pose_[1], cur_pose_[2]);
  }

  RCLCPP_INFO(
      this->get_logger(),
      "\n\nEnd Effector Pose received \nEE starting position is : x: "
      "%f | y: %f | z: %f | \nEE starting rotation is : x: %f | y: "
      "%f | z: %f | w: %f\n",
      ee_starting_pose.pose.position.x, ee_starting_pose.pose.position.y,
      ee_starting_pose.pose.position.z, ee_starting_pose.pose.orientation.x,
      ee_starting_pose.pose.orientation.y, ee_starting_pose.pose.orientation.z,
      ee_starting_pose.pose.orientation.w);

  init_vf_enforcer();

  // Perform impedance loop at 1000 Hz
  controlThread_ = this->create_wall_timer(
      1ms, std::bind(&KeyboardControl::controlThread, this));
  RCLCPP_INFO(this->get_logger(), "\033[0;32mImpedance thread started\033[0m");
}

// This function is called at 1000 Hz
void KeyboardControl::controlThread() {
  readInput();
  // Publish target position
  target_pose_.header.stamp = target_pose_tf_.header.stamp = get_clock()->now();
  target_pose_.header.frame_id = target_pose_tf_.header.frame_id =
      base_link_name_;
  target_pose_tf_.child_frame_id = "haptic_interface_target";

  // computing error
  Eigen::Vector3d error;
  error[0] = x_tilde_new_[0] - ee_starting_pose.pose.position.x;
  error[1] = x_tilde_new_[1] - ee_starting_pose.pose.position.y;
  error[2] = x_tilde_new_[2] - ee_starting_pose.pose.position.z;

  target_pose_.pose.position.x = target_pose_tf_.transform.translation.x =
      ee_starting_pose.pose.position.x + error(0);
  target_pose_.pose.position.y = target_pose_tf_.transform.translation.y =
      ee_starting_pose.pose.position.y + error(1);
  target_pose_.pose.position.z = target_pose_tf_.transform.translation.z =
      ee_starting_pose.pose.position.z + error(2);

  // eigen quat order is w x y z
  Eigen::Quaterniond qStart(
      ee_starting_pose.pose.orientation.w, ee_starting_pose.pose.orientation.x,
      ee_starting_pose.pose.orientation.y, ee_starting_pose.pose.orientation.z);
  Eigen::Quaterniond qCur(cur_pose_[6], cur_pose_[3], cur_pose_[4],
                          cur_pose_[5]);
  Eigen::Quaterniond qEEStart(
      ee_starting_pose.pose.orientation.w, ee_starting_pose.pose.orientation.x,
      ee_starting_pose.pose.orientation.y, ee_starting_pose.pose.orientation.z);

  // RCLCPP_INFO(this->get_logger(),
  //             "Current target position: x: %f | y: %f | z: %f",
  //             target_pose_.pose.position.x, target_pose_.pose.position.y,
  //             target_pose_.pose.position.z);
  // computing the difference between the current orientation and the starting
  // orientation of the haptic device
  Eigen::Quaterniond qDiff = qCur * qStart.conjugate();
  qDiff.normalize();

  // applying delta rotation to the end effector starting orientation
  Eigen::Quaterniond qTarget = qDiff * qEEStart;
  qTarget.normalize();

  target_pose_.pose.orientation.x = target_pose_tf_.transform.rotation.x =
      qTarget.x();
  target_pose_.pose.orientation.y = target_pose_tf_.transform.rotation.y =
      qTarget.y();
  target_pose_.pose.orientation.z = target_pose_tf_.transform.rotation.z =
      qTarget.z();
  target_pose_.pose.orientation.w = target_pose_tf_.transform.rotation.w =
      qTarget.w();

  // send trasnform and publish target pose
  tf_broadcaster_->sendTransform(target_pose_tf_);
  target_pos_publisher_->publish(target_pose_);
}
void KeyboardControl::projectTargetOnSphere(
    Eigen::Vector3d &target_position_vec, double safety_sphere_radius_) {
  target_position_vec =
      target_position_vec.normalized() * safety_sphere_radius_;
}

int main(int argc, char **argv) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Haptic Control node");

  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardControl>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling impedance service:");

  node->startLoop();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}