#include "haptic_control.hpp"

using namespace Eigen;

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

HapticControl::HapticControl(const std::string &name,
                             const std::string &namespace_,
                             const rclcpp::NodeOptions &options)
    : Node(name, namespace_, options) {
  // safety sphere around robot base link to prevent singularity
  this->enable_safety_sphere_ =
      this->get_parameter("enable_safety_sphere").as_bool();
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
    this->safety_box_width_ = this->get_parameter("safety_box_width").as_double();   // x
    this->safety_box_length_ = this->get_parameter("safety_box_length").as_double(); // y
    this->safety_box_height_ = this->get_parameter("safety_box_height").as_double(); // z
  } else {
    this->safety_box_width_ = std::numeric_limits<double>::infinity();
    this->safety_box_length_ = std::numeric_limits<double>::infinity();
    this->safety_box_height_ = std::numeric_limits<double>::infinity();
  }

  this->get_parameter("max_force_", max_force_);
  // safety XYZ position zone -> read from config file
  this->force_scale_ = this->get_parameter("force_scale").as_double();
  this->tool_link_name_ = this->get_parameter("tool_link_name").as_string();
  this->base_link_name_ = this->get_parameter("base_link_name").as_string();
  this->ft_link_name_ = this->get_parameter("ft_link_name").as_string();

  // Create a parameter subscriber that can be used to monitor parameter changes
  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  this->get_parameter("max_force_", max_force_);

  // init wrench msg
  current_wrench_.header.frame_id = base_link_name_;
  current_wrench_.wrench.force.x = 0.0;
  current_wrench_.wrench.force.y = 0.0;
  current_wrench_.wrench.force.z = 0.0;
  current_wrench_.wrench.torque.x = 0.0;
  current_wrench_.wrench.torque.y = 0.0;
  current_wrench_.wrench.torque.z = 0.0;

  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing publishers");
  out_virtuose_status_ =
      this->create_subscription<raptor_api_interfaces::msg::OutVirtuoseStatus>(
          "out_virtuose_status", 1,
          std::bind(&HapticControl::out_virtuose_statusCB, this, _1));
  _out_virtuose_pose_ =
      this->create_subscription<raptor_api_interfaces::msg::OutVirtuosePose>(
          "out_virtuose_pose", 1,
          std::bind(&HapticControl::out_virtuose_pose_CB, this, _1));

  target_pos_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_frame",
                                                              1);
  current_target_pos_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("current_frame",
                                                              1);

  _in_virtuose_force =
      this->create_publisher<raptor_api_interfaces::msg::InVirtuoseForce>(
          "in_virtuose_force", 1);
  // create force/wrench subscriber
  ft_subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "bus0/ft_sensor0/ft_sensor_readings/wrench", 1,
      std::bind(&HapticControl::SetWrenchCB, this, _1));

  client_ = this->create_client<raptor_api_interfaces::srv::VirtuoseImpedance>(
      "virtuose_impedance");

  // Set a callback for parameters updates
  // Safety sphere
  cb_enable_safety_sphere_ = param_subscriber_->add_parameter_callback(
      "enable_safety_sphere", std::bind(&HapticControl::enable_safety_sphere_CB,
                                        this, std::placeholders::_1));
  cb_safety_sphere_radius_ = param_subscriber_->add_parameter_callback(
      "safety_sphere_radius", std::bind(&HapticControl::set_safety_sphere_radius_CB,
                                  this, std::placeholders::_1));
  // Safety box
  cb_enable_safety_box_ = param_subscriber_->add_parameter_callback(
      "enable_safety_box", std::bind(&HapticControl::enable_safety_box_CB,
                                        this, std::placeholders::_1));
  cb_safety_box_width_ = param_subscriber_->add_parameter_callback(
      "safety_box_width", std::bind(&HapticControl::set_safety_box_width_CB,
                                    this, std::placeholders::_1));
  cb_safety_box_length_ = param_subscriber_->add_parameter_callback(
      "safety_box_length", std::bind(&HapticControl::set_safety_box_length_CB,
                                     this, std::placeholders::_1));
  cb_safety_box_height_ = param_subscriber_->add_parameter_callback(
      "safety_box_height", std::bind(&HapticControl::set_safety_box_height_CB,
                                     this, std::placeholders::_1));                                     


  received_haptic_pose_ = false;

  // Initializes the TF2 transform listener and buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // defines the rotation from the robot base frame to the haptic base frame
  Eigen::Quaterniond tmp(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  q_haptic_base_to_robot_base_.x() = tmp.x();
  q_haptic_base_to_robot_base_.y() = tmp.y();
  q_haptic_base_to_robot_base_.z() = tmp.z();
  q_haptic_base_to_robot_base_.w() = tmp.w();

  x_new_ << 0.0, 0.0, 0.0;
  x_old_ << 0.0, 0.0, 0.0;
  x_tilde_new_ << 0.0, 0.0, 0.0;
  x_tilde_old_ << 0.0, 0.0, 0.0;
}

void HapticControl::enable_safety_sphere_CB(const rclcpp::Parameter &p) {
  RCLCPP_INFO(this->get_logger(), "Toggled Safety sphere");
  enable_safety_sphere_ = (bool)p.as_int();
}
void HapticControl::set_safety_sphere_radius_CB(const rclcpp::Parameter &p) {
  RCLCPP_INFO(this->get_logger(),
              "Received an update to the safety sphere radius");
  safety_sphere_radius_ = std::clamp(p.as_double(), 0.0, 2.0);
}
void HapticControl::enable_safety_box_CB(const rclcpp::Parameter &p) {
  RCLCPP_INFO(this->get_logger(), "Toggled Safety box");
  enable_safety_box_ = (bool)p.as_int();
}
void HapticControl::set_safety_box_width_CB(const rclcpp::Parameter &p) {
  RCLCPP_INFO(this->get_logger(), "Received an update to the safety box width");
  safety_box_width_ = std::clamp(p.as_double(), 0.0, 2.0);
}
void HapticControl::set_safety_box_length_CB(const rclcpp::Parameter &p) {
  RCLCPP_INFO(this->get_logger(),
              "Received an update to the safety box length");
  safety_box_length_ = std::clamp(p.as_double(), 0.0, 2.0);
}
void HapticControl::set_safety_box_height_CB(const rclcpp::Parameter &p) {
  RCLCPP_INFO(this->get_logger(),
              "Received an update to the safety box height");
  safety_box_height_ = std::clamp(p.as_double(), 0.0, 2.0);
}

void HapticControl::SetWrenchCB(
    const geometry_msgs::msg::WrenchStamped target_wrench) {
  current_wrench_.header.stamp = target_wrench.header.stamp;
  geometry_msgs::msg::WrenchStamped force;
  force.header.stamp = target_wrench.header.stamp;
  force.wrench.force.x = target_wrench.wrench.force.x;
  force.wrench.force.y = target_wrench.wrench.force.y;
  force.wrench.force.z = target_wrench.wrench.force.z;
  force.wrench.torque.x = target_wrench.wrench.torque.x;
  force.wrench.torque.y = target_wrench.wrench.torque.y;
  force.wrench.torque.z = target_wrench.wrench.torque.z;

  // Map forces from probe frame to haptic base frame (since haptic base frame
  // coincides with robot base frame)
  try {
    auto trans = tf_buffer_->lookupTransform(base_link_name_, ft_link_name_,
                                             tf2::TimePointZero);
    // apply rotation to the force
    tf2::doTransform(force, force, trans);

    current_wrench_.wrench.force.x = force.wrench.force.x;
    current_wrench_.wrench.force.y = force.wrench.force.y;
    current_wrench_.wrench.force.z = force.wrench.force.z;
    current_wrench_.wrench.torque.x = force.wrench.torque.x;
    current_wrench_.wrench.torque.y = force.wrench.torque.y;
    current_wrench_.wrench.torque.z = force.wrench.torque.z;
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                          "F/T sensor pose transform not available: %s",
                          ex.what());
    current_wrench_.wrench.force.x = current_wrench_.wrench.force.y =
        current_wrench_.wrench.force.z = 0.0;
    current_wrench_.wrench.torque.x = current_wrench_.wrench.torque.y =
        current_wrench_.wrench.torque.z = 0.0;
    return;
  }
}
// Callback for topic out_virtuose_pose_
void HapticControl::out_virtuose_pose_CB(
    const raptor_api_interfaces::msg::OutVirtuosePose::SharedPtr msg) {
  if (!received_haptic_pose_) {
    // Store last pose date
    received_haptic_pose_ = true;
    haptic_starting_position_.pose.position.x =
        msg->virtuose_pose.translation.x;
    haptic_starting_position_.pose.position.y =
        msg->virtuose_pose.translation.y;
    haptic_starting_position_.pose.position.z =
        msg->virtuose_pose.translation.z;
    haptic_starting_position_.pose.orientation.x =
        msg->virtuose_pose.rotation.x;
    haptic_starting_position_.pose.orientation.y =
        msg->virtuose_pose.rotation.y;
    haptic_starting_position_.pose.orientation.z =
        msg->virtuose_pose.rotation.z;
    haptic_starting_position_.pose.orientation.w =
        msg->virtuose_pose.rotation.w;
    x_new_ << haptic_starting_position_.pose.position.x,
        haptic_starting_position_.pose.position.y,
        haptic_starting_position_.pose.position.z;
    x_old_ = x_new_;
  }
  pose_date_nanosec_ = msg->header.stamp.nanosec;
  pose_date_sec_ = msg->header.stamp.sec;
  cur_pose_[0] = msg->virtuose_pose.translation.x;
  cur_pose_[1] = msg->virtuose_pose.translation.y;
  cur_pose_[2] = msg->virtuose_pose.translation.z;
  cur_pose_[3] = msg->virtuose_pose.rotation.x;
  cur_pose_[4] = msg->virtuose_pose.rotation.y;
  cur_pose_[5] = msg->virtuose_pose.rotation.z;
  cur_pose_[6] = msg->virtuose_pose.rotation.w;

  x_old_ = x_new_;
  x_new_ << cur_pose_[0], cur_pose_[1], cur_pose_[2];

  // ENFORCE SPHERE SAFETY
  if (enable_safety_sphere_) {
    x_tilde_new_ = x_tilde_old_ + (x_new_ - x_old_);
  } else {
    x_tilde_new_ = x_new_;
  }

  // if (ctr_ % 1000 == 0)
  // {
  // printf("Virtuose pose: %f %f %f %f %f %f %f\n", cur_pose_[0], cur_pose_[1],
  // cur_pose_[2], cur_pose_[3], cur_pose_[4], cur_pose_[5], cur_pose_[6]);
  // }
}

// Callback for topic out_virtuose_status
void HapticControl::out_virtuose_statusCB(
    const raptor_api_interfaces::msg::OutVirtuoseStatus::SharedPtr msg) {
  // Store last status date
  status_date_nanosec_ = msg->header.stamp.nanosec;
  status_date_sec_ = msg->header.stamp.sec;
  status_state_ = msg->state;
  status_button_ = msg->buttons;
}

void HapticControl::call_impedance_service() {
  received_ee_pose_ = false;
  while (!received_ee_pose_) {
    try {
      auto trans = tf_buffer_->lookupTransform(base_link_name_, tool_link_name_,
                                               tf2::TimePointZero);
      ee_starting_position.pose.position.x = trans.transform.translation.x;
      ee_starting_position.pose.position.y = trans.transform.translation.y;
      ee_starting_position.pose.position.z = trans.transform.translation.z;
      ee_starting_position.pose.orientation.x = trans.transform.rotation.x;
      ee_starting_position.pose.orientation.y = trans.transform.rotation.y;
      ee_starting_position.pose.orientation.z = trans.transform.rotation.z;
      ee_starting_position.pose.orientation.w = trans.transform.rotation.w;
      received_ee_pose_ = true;
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "End Effector pose not available: %s", ex.what());
    }
  }
  if (enable_safety_sphere_) {
    Eigen::Vector3d ee_start(ee_starting_position.pose.position.x,
                             ee_starting_position.pose.position.y,
                             ee_starting_position.pose.position.z);
    if (ee_start.norm() > safety_sphere_radius_) {
      RCLCPP_ERROR(this->get_logger(),
                   "ERROR: End effector is outside the safety sphere, please "
                   "move it inside the safety sphere");
      exit(1);
    }
  }
  RCLCPP_INFO(this->get_logger(),
              "\n\nEnd Effector Pose received \nEE starting position is : x: "
              "%f | y: %f | z: %f | \nEE starting rotation is : x: %f | y: "
              "%f | z: %f | w: %f\n",
              ee_starting_position.pose.position.x,
              ee_starting_position.pose.position.y,
              ee_starting_position.pose.position.z,
              ee_starting_position.pose.orientation.x,
              ee_starting_position.pose.orientation.y,
              ee_starting_position.pose.orientation.z,
              ee_starting_position.pose.orientation.w);

  // Request impedance mode

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending impedance request");

  auto imp = std::make_shared<
      raptor_api_interfaces::srv::VirtuoseImpedance::Request>();
  this->get_parameter("channel", imp->channel);
  this->get_parameter("ff_device_ip_address", imp->ff_device_ip_address);
  this->get_parameter("ff_device_param_file", imp->ff_device_param_file);
  this->get_parameter("local_ip_address", imp->local_ip_address);

  imp->base_frame.translation.x = 0.0;
  imp->base_frame.translation.y = 0.0;
  imp->base_frame.translation.z = 0.0;

  imp->base_frame.rotation.x = q_haptic_base_to_robot_base_.x();
  imp->base_frame.rotation.y = q_haptic_base_to_robot_base_.y();
  imp->base_frame.rotation.z = q_haptic_base_to_robot_base_.z();
  imp->base_frame.rotation.w = q_haptic_base_to_robot_base_.w();

  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto result = client_->async_send_request(imp);
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
      1ms, std::bind(&HapticControl::impedanceThread, this));
  RCLCPP_INFO(this->get_logger(), "\033[0;32mImpedance thread started\033[0m");
}

void HapticControl::impedanceThread() {
  if (!received_haptic_pose_) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Haptic pose not available");
    return;
  }
  // Apply the force
  raptor_api_interfaces::msg::InVirtuoseForce force;
  force.header.stamp.nanosec = get_clock()->now().nanoseconds();
  force.header.stamp.sec = get_clock()->now().seconds();
  force.client_id = client__id_;
  // filter noise
  float alpha = 0;
  force.virtuose_force.force.x =
      0.3 * (alpha * old_force_.virtuose_force.force.x +
             (1 - alpha) * current_wrench_.wrench.force.x);
  force.virtuose_force.force.y =
      0.3 * (alpha * old_force_.virtuose_force.force.y +
             (1 - alpha) * current_wrench_.wrench.force.y);
  force.virtuose_force.force.z =
      0.3 * (alpha * old_force_.virtuose_force.force.z +
             (1 - alpha) * current_wrench_.wrench.force.z);
  // torque omitted for control simplicity
  force.virtuose_force.torque.x =
      0.0; // 0.2 * (alpha * old_force_.virtuose_force.torque.x + (1 - alpha)
           // * current_wrench_.wrench.torque.x);
  force.virtuose_force.torque.y =
      0.0; // 0.2 * (alpha * old_force_.virtuose_force.torque.y + (1 - alpha)
           // * current_wrench_.wrench.torque.y);
  force.virtuose_force.torque.z =
      0.0; // 0.2 * (alpha * old_force_.virtuose_force.torque.z + (1 - alpha)
           // * current_wrench_.wrench.torque.z);

  // SAFE ZONE FORCE

  force.virtuose_force.force.x =
      std::clamp(force.virtuose_force.force.x, -6.0, 6.0);
  force.virtuose_force.force.y =
      std::clamp(force.virtuose_force.force.y, -6.0, 6.0);
  force.virtuose_force.force.z =
      std::clamp(force.virtuose_force.force.z, -6.0, 6.0);

  // // SAFE ZONE TORQUE
  // force.virtuose_force.torque.x =
  // std::clamp(force.virtuose_force.torque.x,-0.2,0.2);
  // force.virtuose_force.torque.y =
  // std::clamp(force.virtuose_force.torque.y,-0.2,0.2);
  // force.virtuose_force.torque.z =
  // std::clamp(force.virtuose_force.torque.z,-0.2,0.2);

  // updating old force
  old_force_.virtuose_force.force.x = force.virtuose_force.force.x;
  old_force_.virtuose_force.force.y = force.virtuose_force.force.y;
  old_force_.virtuose_force.force.z = force.virtuose_force.force.z;
  old_force_.virtuose_force.torque.x = force.virtuose_force.torque.x;
  old_force_.virtuose_force.torque.y = force.virtuose_force.torque.y;
  old_force_.virtuose_force.torque.z = force.virtuose_force.torque.z;

  _in_virtuose_force->publish(force);
  ctr_++;
  // uint64_t dt = get_clock()->now().nanoseconds() -
  //               (long long unsigned int)status_date_sec_ * 1000000000U +
  //               (long long unsigned int)status_date_nanosec_;

  // Publish target position
  geometry_msgs::msg::PoseStamped target_pose_, current_pose_;
  target_pose_.header.stamp.nanosec = get_clock()->now().nanoseconds();
  target_pose_.header.stamp.sec = get_clock()->now().seconds();
  target_pose_.header.frame_id = base_link_name_;

  // computing error
  Eigen::Vector3d error;
  error[0] = x_tilde_new_[0] - haptic_starting_position_.pose.position.x;
  error[1] = x_tilde_new_[1] - haptic_starting_position_.pose.position.y;
  error[2] = x_tilde_new_[2] - haptic_starting_position_.pose.position.z;

  // applying rotation from haptic base frame to robot base frame
  // error = q_haptic_base_to_robot_base_.toRotationMatrix() * error;

  target_pose_.pose.position.x =
      ee_starting_position.pose.position.x + error(0);
  target_pose_.pose.position.y =
      ee_starting_position.pose.position.y + error(1);
  target_pose_.pose.position.z =
      ee_starting_position.pose.position.z + error(2);

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

  // eigen quat order is w x y z
  Eigen::Quaterniond qStart(haptic_starting_position_.pose.orientation.w,
                            haptic_starting_position_.pose.orientation.x,
                            haptic_starting_position_.pose.orientation.y,
                            haptic_starting_position_.pose.orientation.z);
  Eigen::Quaterniond qCur(cur_pose_[6], cur_pose_[3], cur_pose_[4],
                          cur_pose_[5]);
  Eigen::Quaterniond qEEStart(ee_starting_position.pose.orientation.w,
                              ee_starting_position.pose.orientation.x,
                              ee_starting_position.pose.orientation.y,
                              ee_starting_position.pose.orientation.z);

  // transform back to quaternion
  // current_pose_.header.frame_id = base_link_name_;
  // current_pose_.pose.position.x = target_pose_.pose.position.x;
  // current_pose_.pose.position.y = target_pose_.pose.position.y;
  // current_pose_.pose.position.z = target_pose_.pose.position.z;

  // // SAFE ZONE POSITION
  // current_pose_.pose.position.x = std::clamp(current_pose_.pose.position.x,
  // min_x_, max_x_); current_pose_.pose.position.y =
  // std::clamp(current_pose_.pose.position.y, min_y_, max_y_);
  // current_pose_.pose.position.z = std::clamp(current_pose_.pose.position.z,
  // min_z_, max_z_);

  // current_pose_.pose.orientation.x = qCur.x();
  // current_pose_.pose.orientation.y = qCur.y();
  // current_pose_.pose.orientation.z = qCur.z();
  // current_pose_.pose.orientation.w = qCur.w();
  // current_target_pos_publisher_->publish(current_pose_);

  // computing the difference between the current orientation and the starting
  // orientation of the haptic device
  Eigen::Quaterniond qDiff = qCur * qStart.conjugate();
  qDiff.normalize();

  // applying delta rotation to the end effector starting orientation
  Eigen::Quaterniond qTarget = qDiff * qEEStart;
  qTarget.normalize();

  target_pose_.pose.orientation.x = qTarget.x();
  target_pose_.pose.orientation.y = qTarget.y();
  target_pose_.pose.orientation.z = qTarget.z();
  target_pose_.pose.orientation.w = qTarget.w();

  target_pos_publisher_->publish(target_pose_);
}
void HapticControl::project_target_on_sphere(
    Eigen::Vector3d &target_position_vec, double safety_sphere_radius_) {
  target_position_vec =
      target_position_vec.normalized() * safety_sphere_radius_;
}
int main(int argc, char **argv) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Haptic Control node");

  rclcpp::init(argc, argv);
  auto node = std::make_shared<HapticControl>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling impedance service:");

  node->call_impedance_service();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}