#include "haptic_control_base.hpp"
using namespace Eigen;

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

geometry_msgs::msg::Point vector3_to_point(
    const geometry_msgs::msg::Vector3 &vec) {
  geometry_msgs::msg::Point point;
  point.x = vec.x;
  point.y = vec.y;
  point.z = vec.z;
  return point;
}

HapticControlBase::HapticControlBase(const std::string &name,
                                     const std::string &namespace_,
                                     const rclcpp::NodeOptions &options)
    : Node(name, namespace_, options) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Starting Haptic Control node with name %s", name.c_str());
  // safety sphere around robot base link to prevent singularity
  this->enable_safety_sphere_ =
      this->get_parameter("enable_safety_sphere").as_bool();
  use_fixtures_ = this->get_parameter("use_fixtures").as_bool();
  if (this->enable_safety_sphere_) {
    this->safety_sphere_radius_ =
        this->get_parameter("safety_sphere_radius").as_double();
    this->safety_sphere_radius_ =
        std::clamp(this->safety_sphere_radius_, 0.0, 2.0);
  } else {
    this->safety_sphere_radius_ = std::numeric_limits<double>::infinity();
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
  this->delay_ = this->get_parameter("delay").as_double();
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
      "target_frame", 1);
  desired_frame_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "desired_frame", 1);
  current_frame_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "current_frame", 1);

  // create force/wrench subscriber
  ft_subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "bus0/ft_sensor0/ft_sensor_readings/wrench", 1,
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
  Eigen::Quaterniond q_haptic_base_to_robot_base_(
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  haptic_device_ =
      std::make_shared<SystemInterface>(q_haptic_base_to_robot_base_);

  // new pose
  x_new_ << 0.0, 0.0, 0.0;
  // old pose
  x_old_ << 0.0, 0.0, 0.0;
  // safe new pose
  x_tilde_new_ << 0.0, 0.0, 0.0;
  // safe old pose
  x_tilde_old_ << 0.0, 0.0, 0.0;
}

void HapticControlBase::init_vf_enforcer() {
  // Init virtual fixture enforcer
  vf_enforcer_ = std::make_shared<VFEnforcer>(
      this->shared_from_this(),
      Eigen::Vector3d(haptic_device_->haptic_current_pose_.pose.position.x,
                      haptic_device_->haptic_current_pose_.pose.position.y,
                      haptic_device_->haptic_current_pose_.pose.position.z));
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
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "End Effector pose not available: %s", ex.what());
  }
}

void HapticControlBase::initialize_haptic_control() {
  received_ee_pose_ = false;
  geometry_msgs::msg::TransformStamped trans;
  while (!received_ee_pose_) {
    get_ee_trans(trans);
  }
  ee_starting_position.pose.position =
      vector3_to_point(trans.transform.translation);
  ee_starting_position.pose.orientation = trans.transform.rotation;
  if (enable_safety_sphere_) {
    Eigen::Vector3d ee_start(ee_starting_position.pose.position.x,
                             ee_starting_position.pose.position.y,
                             ee_starting_position.pose.position.z);
    if (ee_start.norm() > safety_sphere_radius_) {
      RCLCPP_ERROR(this->get_logger(),
                   "ERROR: End effector is outside the safety sphere, please "
                   "move it inside the safety sphere");
      rclcpp::shutdown();
    }
  }
  RCLCPP_INFO(this->get_logger(),
              "\n\nEnd Effector Pose received \nEE starting position is : x: "
              "%f | y: %f | z: %f",
              ee_starting_position.pose.position.x,
              ee_starting_position.pose.position.y,
              ee_starting_position.pose.position.z);
  qEEStart = Eigen::Quaterniond(ee_starting_position.pose.orientation.w,
                                ee_starting_position.pose.orientation.x,
                                ee_starting_position.pose.orientation.y,
                                ee_starting_position.pose.orientation.z);
  // Start the haptic device
  haptic_device_->create_connection();

  while (!haptic_device_->received_haptic_pose_) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Waiting for the haptic device to start publishing");
    rclcpp::spin_some(haptic_device_);
  }
  haptic_device_->start_force_feedback();
  x_new_ << haptic_device_->haptic_starting_pose_.pose.position.x,
      haptic_device_->haptic_starting_pose_.pose.position.y,
      haptic_device_->haptic_starting_pose_.pose.position.z;
  x_old_ = x_tilde_old_ = x_new_;

  // in the case of no delay, the buffer will contain only the last element
  for (int i = 0; i < delay_loop_haptic_; i++) {
    target_pose_buffer_.push(ee_starting_position);
    target_pose_vf_buffer_.push(ee_starting_position);
  }
  for (int i = 0; i < delay_loop_ft_; i++) {
    wrench_buffer_.push(current_wrench_);
  }

  if (use_fixtures_) {
    init_vf_enforcer();
  }
  // Perform impedance loop at haptic_control_rate Hz
  control_thread_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / haptic_control_rate_),
      std::bind(&HapticControlBase::control_thread, this));
  RCLCPP_INFO(this->get_logger(), "\033[0;32mControl thread started\033[0m");
}

Eigen::Vector3d HapticControlBase::compute_position_error() {
  Eigen::Vector3d error;
  error[0] =
      x_tilde_new_[0] - haptic_device_->haptic_starting_pose_.pose.position.x;
  error[1] =
      x_tilde_new_[1] - haptic_device_->haptic_starting_pose_.pose.position.y;
  error[2] =
      x_tilde_new_[2] - haptic_device_->haptic_starting_pose_.pose.position.z;
  return error;
}
Eigen::Quaterniond HapticControlBase::compute_orientation_error() {
  Eigen::Quaterniond qStart(
      haptic_device_->haptic_starting_pose_.pose.orientation.w,
      haptic_device_->haptic_starting_pose_.pose.orientation.x,
      haptic_device_->haptic_starting_pose_.pose.orientation.y,
      haptic_device_->haptic_starting_pose_.pose.orientation.z);
  Eigen::Quaterniond qCur(
      haptic_device_->haptic_current_pose_.pose.orientation.w,
      haptic_device_->haptic_current_pose_.pose.orientation.x,
      haptic_device_->haptic_current_pose_.pose.orientation.y,
      haptic_device_->haptic_current_pose_.pose.orientation.z);

  // computing the difference between the current orientation and the starting
  // orientation of the haptic device
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
  // Update the haptic device pose
  if (enable_safety_sphere_) {
    x_tilde_new_ = x_tilde_old_ + (x_new_ - x_old_);
  } else {
    x_tilde_new_ = x_new_;
  }
  // Applies the feedback force
  // retrieve the oldest wrench in the buffer
  haptic_device_->update_target_wrench(wrench_buffer_.peek());
  // Computes errors
  Eigen::Vector3d position_error = compute_position_error();
  Eigen::Quaterniond orientation_error = compute_orientation_error();

  // Updates the target pose metadata
  target_pose_.header.stamp = target_pose_tf_.header.stamp = get_clock()->now();
  target_pose_.header.frame_id = target_pose_tf_.header.frame_id =
      base_link_name_;
  target_pose_tf_.child_frame_id = "haptic_interface_target";

  // Applies delta position to the end effector starting position
  target_pose_.pose.position.x = target_pose_tf_.transform.translation.x =
      ee_starting_position.pose.position.x + position_error(0);
  target_pose_.pose.position.y = target_pose_tf_.transform.translation.y =
      ee_starting_position.pose.position.y + position_error(1);
  target_pose_.pose.position.z = target_pose_tf_.transform.translation.z =
      ee_starting_position.pose.position.z + position_error(2);

  // Applies delta rotation to the end effector starting orientation
  Eigen::Quaterniond target_orientation = orientation_error * qEEStart;
  target_orientation.normalize();
  target_pose_.pose.orientation.x = target_pose_tf_.transform.rotation.x =
      target_orientation.x();
  target_pose_.pose.orientation.y = target_pose_tf_.transform.rotation.y =
      target_orientation.y();
  target_pose_.pose.orientation.z = target_pose_tf_.transform.rotation.z =
      target_orientation.z();
  target_pose_.pose.orientation.w = target_pose_tf_.transform.rotation.w =
      target_orientation.w();

  // Enforce safety sphere zone
  if (enable_safety_sphere_) {
    Eigen::Vector3d target_position_vec(target_pose_.pose.position.x,
                                        target_pose_.pose.position.y,
                                        target_pose_.pose.position.z);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Current distance: %f, safety sphere radius: %f",
                         target_position_vec.norm(), safety_sphere_radius_);
    if (target_position_vec.norm() > safety_sphere_radius_) {
      // project the target on the safety sphere to prevent singularities
      project_target_on_sphere(target_position_vec, safety_sphere_radius_);
    } else {
      // update the old pose only if the target is within the safety sphere
      x_tilde_old_ = x_tilde_new_;
    }
  }
  target_pose_buffer_.push(target_pose_);

  if (use_fixtures_) {
    Eigen::Vector3d x_desired(target_pose_.pose.position.x,
                              target_pose_.pose.position.y,
                              target_pose_.pose.position.z);
    auto delta_x = vf_enforcer_->enforce_vf(x_desired);
    target_pose_vf_.header.stamp = this->get_clock()->now();
    target_pose_vf_.header.frame_id = base_link_name_;
    target_pose_vf_.header.frame_id = target_pose_.header.frame_id;
    target_pose_vf_.pose.position.x =
        old_target_pose_vf_.pose.position.x + delta_x[0];
    target_pose_vf_.pose.position.y =
        old_target_pose_vf_.pose.position.y + delta_x[1];
    target_pose_vf_.pose.position.z =
        old_target_pose_vf_.pose.position.z + delta_x[2];
    target_pose_vf_.pose.orientation = target_pose_.pose.orientation;

    old_target_pose_vf_ = target_pose_vf_;

    target_pose_vf_buffer_.push(target_pose_vf_);
  }

  // Publish the current ee pose
  geometry_msgs::msg::TransformStamped ee_pose;
  get_ee_trans(ee_pose);
  geometry_msgs::msg::PoseStamped ee_pose_msg;
  ee_pose_msg.header.stamp = ee_pose.header.stamp;
  ee_pose_msg.header.frame_id = base_link_name_;
  ee_pose_msg.pose.position = vector3_to_point(ee_pose.transform.translation);
  ee_pose_msg.pose.orientation = ee_pose.transform.rotation;
  current_frame_pub_->publish(ee_pose_msg);
  if (this->use_fixtures_) {
    // Publish the target pose
    target_frame_pub_->publish(target_pose_vf_buffer_.peek());
  } else {
    target_frame_pub_->publish(target_pose_buffer_.peek());
  }
  tf_broadcaster_->sendTransform(target_pose_tf_);
  // Publish the desired pose
  desired_frame_pub_->publish(target_pose_);

  // Update the old pose
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