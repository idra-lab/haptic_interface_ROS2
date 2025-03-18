/*
Teleoperation node using haptic device and force feedback which implements the
mesh virtual fixtures explained in the paper
https://ieeexplore.ieee.org/document/9341590/
@Author: Davide Nardi
*/
#include "haptic_control_base.hpp"
using namespace Eigen;

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

HapticControlBase::HapticControlBase(const std::string &name,
                                     const std::string &namespace_,
                                     const rclcpp::NodeOptions &options)
    : Node(name, namespace_, options) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Starting Haptic Control node with name %s", name.c_str());
  // safety sphere around robot base link to prevent singularity
  this->enable_safety_sphere_ =
      this->get_parameter("enable_safety_sphere").as_bool();
  RCLCPP_INFO(this->get_logger(), "Safety sphere enabled: %d",
              this->enable_safety_sphere_);
  // use virtual fixtures to constraint the robot over a mesh
  use_fixtures_ = this->get_parameter("use_fixtures").as_bool();
  if (use_fixtures_) {
    RCLCPP_WARN(this->get_logger(), "Using Virtual Fixtures");
  }
  // use conic barrier function to constraint the robot orientation within a
  // cone
  use_ccbf_ = this->get_parameter("vf_parameters.use_ccbf").as_bool();
  if (use_fixtures_ && use_ccbf_) {
    std::vector<double> thetas_list(3), q_ref_list(4);
    thetas_list = this->get_parameter("vf_parameters.ccbf_params.ccbf_thetas")
                      .as_double_array();
    thetas_ = Eigen::Vector3d(thetas_list[0], thetas_list[1], thetas_list[2]);

    RCLCPP_WARN(this->get_logger(),
                "Using Conic Barrier Function: \nthetas: %f %f %f", thetas_(0),
                thetas_(1), thetas_(2));
    use_initial_conf_as_q_ref_ =
        this->get_parameter(
                "vf_parameters.ccbf_params.use_initial_conf_as_q_ref")
            .as_bool();
    if (!use_initial_conf_as_q_ref_) {
      q_ref_list = this->get_parameter("vf_parameters.ccbf_params.ccbf_q_ref")
                       .as_double_array();
      q_ref_ = Eigen::Quaterniond(q_ref_list[3], q_ref_list[0], q_ref_list[1],
                                  q_ref_list[2])
                   .normalized();
      RCLCPP_WARN(
          this->get_logger(),
          "Provided reference configuration: x: %f | y: %f | z: %f | w: %f",
          q_ref_.x(), q_ref_.y(), q_ref_.z(), q_ref_.w());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Using initial configuration as reference orientation");
    }
  }
  if (enable_safety_sphere_) {
    safety_sphere_radius_ =
        this->get_parameter("safety_sphere_radius").as_double();
    safety_sphere_radius_ = std::clamp(safety_sphere_radius_, 0.0, 2.0);
  } else {
    safety_sphere_radius_ = std::numeric_limits<double>::infinity();
  }
  // safety box dimension
  this->enable_safety_box_ = this->get_parameter("enable_safety_box").as_bool();
  if (this->enable_safety_box_) {
    // not implemented yet
    this->safety_box_width_ =
        this->get_parameter("safety_box_width").as_double();  // x
    this->safety_box_length_ =
        this->get_parameter("safety_box_length").as_double();  // y
    this->safety_box_height_ =
        this->get_parameter("safety_box_height").as_double();  // z
  } else {
    this->safety_box_width_ = std::numeric_limits<double>::infinity();
    this->safety_box_length_ = std::numeric_limits<double>::infinity();
    this->safety_box_height_ = std::numeric_limits<double>::infinity();
  }
  this->get_parameter("ft_feedback_topic_name", ft_feedback_topic_name_);
  this->get_parameter("target_frame_topic_name", target_frame_topic_name_);
  this->get_parameter("max_force", max_force_);
  this->force_scale_ = this->get_parameter("force_scale").as_double();
  this->tool_link_name_ = this->get_parameter("tool_link_name").as_string();
  this->base_link_name_ = this->get_parameter("base_link_name").as_string();
  this->ft_link_name_ = this->get_parameter("ft_link_name").as_string();
  this->haptic_control_rate_ =
      this->get_parameter("haptic_control_rate").as_double();
  this->ft_sensor_rate_ = this->get_parameter("ft_sensor_rate").as_double();

  // delay simulation
  delay_loop_haptic_ = delay_loop_ft_ = 1;
  this->delay_ = this->get_parameter("simulated_delay").as_double();

  this->delay_ = std::clamp(this->delay_, 0.0, 10.0);
  if (std::abs(this->delay_) > 1e-6) {
    // compute the number of control loop of delay (haptic run at 1kHz)
    this->delay_loop_haptic_ =
        (int)std::ceil((this->delay_) * haptic_control_rate_);
    this->delay_loop_ft_ = (int)std::ceil((this->delay_) * ft_sensor_rate_);
    RCLCPP_WARN(this->get_logger(),
                "A Delay of %f seconds is set, which cooresponds to a %d "
                "control loop delay",
                this->delay_, this->delay_loop_haptic_);
  }
  wrench_buffer_.initialize(delay_loop_ft_);
  target_pose_buffer_.initialize(delay_loop_haptic_);
  target_pose_vf_buffer_.initialize(delay_loop_haptic_);

  // Create a parameter subscriber that can be used to monitor parameter changes
  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  this->get_parameter("max_force", max_force_);

  // init wrench msg
  current_wrench_.header.frame_id = base_link_name_;
  current_wrench_.wrench.force.x = 0.0;
  current_wrench_.wrench.force.y = 0.0;
  current_wrench_.wrench.force.z = 0.0;
  current_wrench_.wrench.torque.x = 0.0;
  current_wrench_.wrench.torque.y = 0.0;
  current_wrench_.wrench.torque.z = 0.0;

  target_frame_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      target_frame_topic_name_, 1);
  desired_frame_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "desired_frame", 1);
  current_frame_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "current_frame", 1);

  // create force/wrench subscriber
  ft_subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      ft_feedback_topic_name_, 1,
      std::bind(&HapticControlBase::store_wrench, this, _1));

  // Set a callback for parameters updates
  // Safety sphere
  cb_enable_safety_sphere_ = param_subscriber_->add_parameter_callback(
      "enable_safety_sphere",
      std::bind(&HapticControlBase::enable_safety_sphere_CB, this,
                std::placeholders::_1));
  cb_safety_sphere_radius_ = param_subscriber_->add_parameter_callback(
      "safety_sphere_radius",
      std::bind(&HapticControlBase::set_safety_sphere_radius_CB, this,
                std::placeholders::_1));
  // Safety box
  cb_enable_safety_box_ = param_subscriber_->add_parameter_callback(
      "enable_safety_box", std::bind(&HapticControlBase::enable_safety_box_CB,
                                     this, std::placeholders::_1));
  cb_safety_box_width_ = param_subscriber_->add_parameter_callback(
      "safety_box_width", std::bind(&HapticControlBase::set_safety_box_width_CB,
                                    this, std::placeholders::_1));
  cb_safety_box_length_ = param_subscriber_->add_parameter_callback(
      "safety_box_length",
      std::bind(&HapticControlBase::set_safety_box_length_CB, this,
                std::placeholders::_1));
  cb_safety_box_height_ = param_subscriber_->add_parameter_callback(
      "safety_box_height",
      std::bind(&HapticControlBase::set_safety_box_height_CB, this,
                std::placeholders::_1));

  // Initializes the TF2 transform listener and buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // defines the rotation from the robot base frame to the haptic base frame
  std::vector<double> base_frame_rot;
  this->get_parameter("base_frame_rotation", base_frame_rot);
  Eigen::Quaterniond q_haptic_base_to_robot_base_(
      base_frame_rot[3], base_frame_rot[0], base_frame_rot[1],
      base_frame_rot[2]);

  // Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  RCLCPP_INFO(
      this->get_logger(),
      "Haptic base to robot base rotation: x: %f | y: %f | z: %f | w: "
      "%f",
      q_haptic_base_to_robot_base_.x(), q_haptic_base_to_robot_base_.y(),
      q_haptic_base_to_robot_base_.z(), q_haptic_base_to_robot_base_.w());
  haptic_device_ =
      std::make_shared<SystemInterface>(q_haptic_base_to_robot_base_);

  // new haptic pose
  x_new_ << 0.0, 0.0, 0.0;
  // old haptic pose
  x_old_ << 0.0, 0.0, 0.0;
  // safe new haptic pose
  x_tilde_new_ << 0.0, 0.0, 0.0;
  // safe old haptic pose
  x_tilde_old_ << 0.0, 0.0, 0.0;
}

void HapticControlBase::init_vf_enforcer() {
  // Init virtual fixture enforcer
  vf_enforcer_ = std::make_shared<VFEnforcer>(
      this->shared_from_this(), eigenFromRosPoint(target_pose_.pose.position),
      this->base_link_name_);
  RCLCPP_INFO(this->get_logger(),
              "\033[0;32mVirtual fixture enforcer initialized\033[0m");
}

void HapticControlBase::enable_safety_sphere_CB(const rclcpp::Parameter &p) {
  RCLCPP_INFO(this->get_logger(), "Toggled Safety sphere");
  enable_safety_sphere_ = (bool)p.as_int();
}
void HapticControlBase::set_safety_sphere_radius_CB(
    const rclcpp::Parameter &p) {
  RCLCPP_INFO(this->get_logger(),
              "Received an update to the safety sphere radius");
  safety_sphere_radius_ = std::clamp(p.as_double(), 0.0, 2.0);
}
void HapticControlBase::enable_safety_box_CB(const rclcpp::Parameter &p) {
  RCLCPP_INFO(this->get_logger(), "Toggled Safety box");
  enable_safety_box_ = (bool)p.as_int();
}
void HapticControlBase::set_safety_box_width_CB(const rclcpp::Parameter &p) {
  RCLCPP_INFO(this->get_logger(), "Received an update to the safety box width");
  safety_box_width_ = std::clamp(p.as_double(), 0.0, 2.0);
}
void HapticControlBase::set_safety_box_length_CB(const rclcpp::Parameter &p) {
  RCLCPP_INFO(this->get_logger(),
              "Received an update to the safety box length");
  safety_box_length_ = std::clamp(p.as_double(), 0.0, 2.0);
}
void HapticControlBase::set_safety_box_height_CB(const rclcpp::Parameter &p) {
  RCLCPP_INFO(this->get_logger(),
              "Received an update to the safety box height");
  safety_box_height_ = std::clamp(p.as_double(), 0.0, 2.0);
}

void HapticControlBase::store_wrench(
    const geometry_msgs::msg::WrenchStamped target_wrench) {
  current_wrench_.header.stamp = target_wrench.header.stamp;

  geometry_msgs::msg::WrenchStamped wrench_stamped;
  wrench_stamped.header.stamp = target_wrench.header.stamp;
  wrench_stamped.wrench = target_wrench.wrench;
  // Map forces from probe frame to haptic base frame (since haptic base frame
  // coincides with robot base frame)
  try {
    auto trans = tf_buffer_->lookupTransform(base_link_name_, ft_link_name_,
                                             tf2::TimePointZero);
    // apply rotation to the force
    tf2::doTransform(wrench_stamped, wrench_stamped, trans);
    current_wrench_.wrench = wrench_stamped.wrench;
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "F/T sensor pose transform not available: %s",
                          ex.what());
    current_wrench_.wrench = geometry_msgs::msg::Wrench();
  }
  wrench_buffer_.push(current_wrench_);
}

void HapticControlBase::get_ee_trans(
    geometry_msgs::msg::TransformStamped &trans) {
  try {
    trans = tf_buffer_->lookupTransform(base_link_name_, tool_link_name_,
                                        tf2::TimePointZero);
    received_ee_pose_ = true;
    last_robot_pose_update_time_ = trans.header.stamp;
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "End Effector pose not available: %s", ex.what());
  }
}

void HapticControlBase::initialize_haptic_control() {
  received_ee_pose_ = false;
  geometry_msgs::msg::TransformStamped trans;

  // Wait until end-effector pose is received
  while (!received_ee_pose_ && rclcpp::ok()) {
    get_ee_trans(trans);
  }

  // Convert transform to position and orientation
  ee_starting_position.pose.position =
      vector3_to_point(trans.transform.translation);
  ee_starting_position.pose.orientation = trans.transform.rotation;

  // Safety sphere check
  if (enable_safety_sphere_) {
    Eigen::Vector3d ee_start =
        eigenFromRosPoint(ee_starting_position.pose.position);
    if (ee_start.norm() > safety_sphere_radius_) {
      RCLCPP_ERROR(this->get_logger(),
                   "ERROR: End effector is outside the safety sphere, please "
                   "move it inside.");
      rclcpp::shutdown();
    }
  }

  // Log starting position
  RCLCPP_INFO(this->get_logger(),
              "\n\nEnd Effector Pose received \nEE starting position is : x: "
              "%f | y: %f | z: %f",
              ee_starting_position.pose.position.x,
              ee_starting_position.pose.position.y,
              ee_starting_position.pose.position.z);

  // Convert starting orientation
  qEEStart_ = rosQuatToEigen(ee_starting_position.pose.orientation);

  // get euler to check if the current orientation is inside cbf cones
  if (use_fixtures_ && use_ccbf_) {
    if (use_initial_conf_as_q_ref_) {
      q_ref_ = qEEStart_;
      RCLCPP_WARN(
          this->get_logger(),
          "Initial configuration quaternion: x: %f | y: %f | z: %f | w: %f",
          q_ref_.x(), q_ref_.y(), q_ref_.z(), q_ref_.w());
    }
    Eigen::Vector3d euler_angles =
        qEEStart_.toRotationMatrix().eulerAngles(0, 1, 2);
    if (!conic_cbf::is_inside_cone(qEEStart_, q_ref_, thetas_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Current orientation is not inside the cone, consider "
                   "changing 'ccbf_q_ref' or 'ccbf_thetas' parameters");
      rclcpp::shutdown();
    }
  }
  // Start haptic device connection
  haptic_device_->create_connection();
  RCLCPP_INFO(this->get_logger(),
              "\033[0;32mHaptic device connection started\033[0m");
  // Wait until haptic device starts publishing
  while (!haptic_device_->received_haptic_pose_) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Waiting for the haptic device to start publishing");
    rclcpp::spin_some(haptic_device_);
  }

  // Initialize position vectors
  x_new_ =
      eigenFromRosPoint(haptic_device_->haptic_starting_pose_.pose.position);
  x_old_ = x_tilde_old_ = x_new_;
  q_new_ = q_old_ = qEEStart_;

  // Initialize target poses
  target_pose_vf_ = target_pose_ = ee_starting_position;

  // Fill buffers for delay handling
  for (int i = 0; i < delay_loop_haptic_; i++) {
    target_pose_buffer_.push(ee_starting_position);
    target_pose_vf_buffer_.push(ee_starting_position);
  }
  for (int i = 0; i < delay_loop_ft_; i++) {
    wrench_buffer_.push(current_wrench_);
  }

  // Initialize virtual fixtures if enabled
  if (use_fixtures_) {
    init_vf_enforcer();
  }

  // Start control thread at `haptic_control_rate_` Hz
  control_thread_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / haptic_control_rate_),
      std::bind(&HapticControlBase::control_thread, this));

  RCLCPP_INFO(this->get_logger(), "\033[0;32mControl thread started\033[0m");

  // Start force feedback
  haptic_device_->start_force_feedback();
  RCLCPP_INFO(this->get_logger(), "\033[0;32mForce feedback started\033[0m");
}
Eigen::Vector3d HapticControlBase::compute_position_error() {
  Eigen::Vector3d haptic_starting_position =
      eigenFromRosPoint(haptic_device_->haptic_starting_pose_.pose.position);

  return x_tilde_new_ - haptic_starting_position;
}

Eigen::Quaterniond HapticControlBase::compute_orientation_error() {
  Eigen::Quaterniond qStart =
      rosQuatToEigen(haptic_device_->haptic_starting_pose_.pose.orientation);
  Eigen::Quaterniond qCur =
      rosQuatToEigen(haptic_device_->haptic_current_pose_.pose.orientation);

  // Compute the difference between current and starting orientation
  Eigen::Quaterniond qDiff = qCur * qStart.conjugate();
  qDiff.normalize();

  return qDiff;
}

// This function is called at 1000 Hz

void HapticControlBase::control_thread() {
  if (!haptic_device_->received_haptic_pose_) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Haptic pose not available");
    return;
  }

  // Robot data consistency check
  if (!first_control_loop_ &&
      (this->get_clock()->now() - last_robot_pose_update_time_).seconds() >
          2.0) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Robot pose not updated within the last 2 seconds, shutting down");
    rclcpp::shutdown();
  }
  // else{
  //   RCLCPP_INFO(this->get_logger(), "Robot pose updated after %f seconds",
  //               (this->get_clock()->now() - last_robot_pose_update_time_).seconds());
  // }
  first_control_loop_ = false;

  // Update haptic device pose with safety sphere check
  x_tilde_new_ =
      enable_safety_sphere_ ? x_tilde_old_ + (x_new_ - x_old_) : x_new_;

  // Apply feedback force
  haptic_device_->update_target_wrench(wrench_buffer_.peek());

  // Compute errors
  Eigen::Vector3d position_error = compute_position_error();
  Eigen::Quaterniond orientation_error = compute_orientation_error();

  // Update target pose metadata
  target_pose_.header.stamp = target_pose_tf_.header.stamp = get_clock()->now();
  target_pose_.header.frame_id = target_pose_tf_.header.frame_id =
      base_link_name_;
  target_pose_tf_.child_frame_id = "haptic_interface_target";

  // Apply delta position
  target_pose_.pose.position.x =
      ee_starting_position.pose.position.x + position_error.x();
  target_pose_.pose.position.y =
      ee_starting_position.pose.position.y + position_error.y();
  target_pose_.pose.position.z =
      ee_starting_position.pose.position.z + position_error.z();
  target_pose_tf_.transform.translation =
      point_to_vector3(target_pose_.pose.position);

  // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
  //                      "Target_position: %f %f %f",
  //                      target_pose_.pose.position.x,
  //                       target_pose_.pose.position.y,
  //                       target_pose_.pose.position.z);

  // Apply delta rotation
  Eigen::Quaterniond target_orientation = orientation_error * qEEStart_;
  target_orientation.normalize();
  target_pose_.pose.orientation = target_pose_tf_.transform.rotation =
      eigenToRosQuat(target_orientation);

  // Enforce safety sphere constraints
  if (enable_safety_sphere_) {
    Eigen::Vector3d target_position_vec(target_pose_.pose.position.x,
                                        target_pose_.pose.position.y,
                                        target_pose_.pose.position.z);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Current distance: %f, safety sphere radius: %f",
                         target_position_vec.norm(), safety_sphere_radius_);

    if (target_position_vec.norm() > safety_sphere_radius_) {
      project_target_on_sphere(target_position_vec, safety_sphere_radius_);
    } else {
      x_tilde_old_ = x_tilde_new_;
    }
  }

  target_pose_buffer_.push(target_pose_);

  // Retrieve end-effector pose
  geometry_msgs::msg::TransformStamped ee_pose;
  get_ee_trans(ee_pose);

  if (use_fixtures_) {
    Eigen::Vector3d x_desired(target_pose_.pose.position.x,
                              target_pose_.pose.position.y,
                              target_pose_.pose.position.z);

    target_pose_vf_.header.stamp = this->get_clock()->now();
    target_pose_vf_.header.frame_id = base_link_name_;
    // // Apply virtual fixture constraints to position
    Eigen::Vector3d x_filtered = vf_enforcer_->enforce_vf(x_desired);
    target_pose_vf_.pose.position = eigenToRosPoint(x_filtered);
    target_pose_vf_.pose.orientation = target_pose_.pose.orientation;
  }
  if (use_ccbf_) {
    // // Apply virtual fixture constraints to orientation
    q_new_ = target_orientation;

    auto q_opt =
        conic_cbf::cbfOrientFilter(q_ref_, q_old_, q_new_, thetas_, 0.001);
    // RCLCPP_INFO(this->get_logger(), "q_old: %f %f %f %f, q_opt: %f %f %f %f",
    //             q_old_.x(), q_old_.y(), q_old_.z(), q_old_.w(), q_opt.x(),
    //             q_opt.y(), q_opt.z(), q_opt.w());
    q_old_ = q_opt;
    target_pose_vf_.pose.orientation = eigenToRosQuat(q_opt);
  }
  target_pose_vf_buffer_.push(target_pose_vf_);

// Publish current end-effector pose
geometry_msgs::msg::PoseStamped ee_pose_msg;
ee_pose_msg.header.stamp = ee_pose.header.stamp;
ee_pose_msg.header.frame_id = base_link_name_;
ee_pose_msg.pose.position = vector3_to_point(ee_pose.transform.translation);
ee_pose_msg.pose.orientation = ee_pose.transform.rotation;
current_frame_pub_->publish(ee_pose_msg);

if (use_fixtures_ || use_ccbf_) {
  // Publish filtered and desired pose
  target_frame_pub_->publish(target_pose_vf_buffer_.peek());
  desired_frame_pub_->publish(target_pose_);
} else {
  // Publish target pose
  target_frame_pub_->publish(target_pose_buffer_.peek());
}

tf_broadcaster_->sendTransform(target_pose_tf_);

// Update old pose
x_old_ = x_new_;
x_new_ << haptic_device_->haptic_current_pose_.pose.position.x,
    haptic_device_->haptic_current_pose_.pose.position.y,
    haptic_device_->haptic_current_pose_.pose.position.z;
}
void HapticControlBase::project_target_on_sphere(
    Eigen::Vector3d &target_position_vec, double safety_sphere_radius_) {
  target_position_vec =
      target_position_vec.normalized() * safety_sphere_radius_;
}

int main(int argc, char **argv) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting VF Control node");

  rclcpp::init(argc, argv);
  auto node = std::make_shared<HapticControlBase>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling impedance service:");

  node->initialize_haptic_control();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(node->haptic_device_);
  executor.spin();
  return 0;
}