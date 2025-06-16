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
                                 const rclcpp::NodeOptions &options)
    : Node(name, options) {
  // safety sphere around robot base link to prevent singularity
  this->tool_link_name_ = this->get_parameter("tool_link_name").as_string();
  this->base_link_name_ = this->get_parameter("base_link_name").as_string();
  this->tool_vis_radius_ = this->get_parameter("tool_vis_radius").as_double();
  this->use_vf_ = this->get_parameter("use_fixtures").as_bool();

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
  thetas << 0.4, 0.4, 0.4;
  x_tilde_new_ << 0.0, 0.0, 0.0;
  RCLCPP_INFO(this->get_logger(), "Starting Keyboard Control node with name %s",
              name.c_str());
}
void KeyboardControl::init_vf_enforcer() {
  // Init virtual fixture enforcer
  vf_enforcer_ = std::make_shared<VFEnforcer>(
      this->shared_from_this(),
      Eigen::Vector3d(cur_pose_[0], cur_pose_[1], cur_pose_[2]),
      this->base_link_name_, this->tool_vis_radius_, visualizer_);
}

void KeyboardControl::readInput() {
  key = getch();
  euler_angles_ = q_new.toRotationMatrix().eulerAngles(0, 1, 2);
  auto tmp = euler_angles_;
  if (key != -1) {
    // Grab the direction data
    if (key == 'w')
      cur_pose_[0] += motion_scale_l;
    else if (key == 's')
      cur_pose_[0] -= motion_scale_l;
    else if (key == 'a')
      cur_pose_[1] += motion_scale_l;
    else if (key == 'd')
      cur_pose_[1] -= motion_scale_l;
    else if (key == 'q')
      cur_pose_[2] += motion_scale_l;
    else if (key == 'e')
      cur_pose_[2] -= motion_scale_l;
    else if (key == 'i')
      euler_angles_[0] += motion_scale_a;
    else if (key == 'k')
      euler_angles_[0] -= motion_scale_a;
    else if (key == 'j')
      euler_angles_[1] += motion_scale_a;
    else if (key == 'l')
      euler_angles_[1] -= motion_scale_a;
    else if (key == 'u')
      euler_angles_[2] += motion_scale_a;
    else if (key == 'o')
      euler_angles_[2] -= motion_scale_a;
    else {
      RCLCPP_INFO(this->get_logger(), "Invalid key");
    }
  }
  RCLCPP_INFO(this->get_logger(),
              "Current euler angles: roll: %f | pitch: %f | yaw: %f ||| Target "
              "euler angles: roll: %f | pitch: %f | yaw: %f",
              tmp[0], tmp[1], tmp[2], euler_angles_[0], euler_angles_[1],
              euler_angles_[2]);
  x_new_ << cur_pose_[0], cur_pose_[1], cur_pose_[2];
  q_new = Eigen::Quaterniond(
              Eigen::AngleAxisd(euler_angles_[0], Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxisd(euler_angles_[1], Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(euler_angles_[2], Eigen::Vector3d::UnitZ()))
              .normalized();
  if (use_vf_) {
    // x_tilde_new_ = vf_enforcer_->enforce_vf(x_new_);
    auto q_opt = conic_cbf::cbfOrientFilter(q_init, q_old, q_new, thetas);
    q_old = q_opt;
  } else {
    x_tilde_new_ = x_new_;
    q_old = q_new;
  }
}

void KeyboardControl::init_control() {
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
    x_tilde_new_ = x_new_ =
        Eigen::Vector3d(cur_pose_[0], cur_pose_[1], cur_pose_[2]);
    q_new = q_old = q_init =
        Eigen::Quaterniond(ee_starting_pose.pose.orientation.w,
                           ee_starting_pose.pose.orientation.x,
                           ee_starting_pose.pose.orientation.y,
                           ee_starting_pose.pose.orientation.z)
            .normalized();
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

  visualizer_ = std::make_shared<Visualizer>(this->shared_from_this(),
                                             this->base_link_name_);
  RCLCPP_INFO(this->get_logger(), "Visualizer initialized");
  if (use_vf_) {
    init_vf_enforcer();
    RCLCPP_INFO(this->get_logger(), "VF enforcer initialized");
  }

  controlThread_ = this->create_wall_timer(
      1ms, std::bind(&KeyboardControl::controlThread, this));
  RCLCPP_INFO(this->get_logger(), "Impedance thread started");
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

  target_pose_.pose.orientation.x = target_pose_tf_.transform.rotation.x =
      q_old.x();
  target_pose_.pose.orientation.y = target_pose_tf_.transform.rotation.y =
      q_old.y();
  target_pose_.pose.orientation.z = target_pose_tf_.transform.rotation.z =
      q_old.z();
  target_pose_.pose.orientation.w = target_pose_tf_.transform.rotation.w =
      q_old.w();

  // send trasnform and publish target pose
  tf_broadcaster_->sendTransform(target_pose_tf_);
  //   RCLCPP_INFO(this->get_logger(),
  //               "Current target position: x: %f | y: %f | z: %f",
  //               target_pose_.pose.position.x, target_pose_.pose.position.y,
  //               target_pose_.pose.position.z);
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
  auto node = std::make_shared<KeyboardControl>("keyboard_control");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Start loop only after the node is properly added to the executor
  node->init_control();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Spinnig the node");
  executor.spin();  // Ensure proper execution context

  rclcpp::shutdown();
  return 0;
}