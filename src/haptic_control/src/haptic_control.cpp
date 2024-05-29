#include "haptic_control.hpp"

using namespace Eigen;

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

HapticControl::HapticControl(const std::string &name,
                             const std::string &namespace_,
                             const rclcpp::NodeOptions &options)
    : Node(name, namespace_, options)
{
  this->declare_parameter("scaling_factor", 1.0);

  // safety XYZ position zone -> default
  this->use_bounding_box = this->get_parameter("use_bounding_box").as_bool();
  // safety XYZ position zone -> default
  this->use_limits = this->get_parameter("use_limits").as_bool();

  // this->declare_parameter("min_x", -1.0);
  this->min_x = this->get_parameter("min_x").as_double();
  // this->declare_parameter("max_x", 1.0);
  this->max_x = this->get_parameter("max_x").as_double();
  // this->declare_parameter("min_y", -1.0);
  this->min_y = this->get_parameter("min_y").as_double();
  // this->declare_parameter("max_y", 1.0);
  this->max_y = this->get_parameter("max_y").as_double();
  // this->declare_parameter("min_z", -1.0);
  this->min_z = this->get_parameter("min_z").as_double();
  // this->declare_parameter("max_z", 1.0);
  this->max_z = this->get_parameter("max_z").as_double();

  this->get_parameter("max_force", max_force);
  // safety XYZ position zone -> read from config file
  this->force_scale_ = this->get_parameter("force_scale").as_double();
  this->tool_link_name_ = this->get_parameter("tool_link_name").as_string();
  this->base_link_name_ = this->get_parameter("base_link_name").as_string();
  // Create a parameter subscriber that can be used to monitor parameter changes
  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  this->get_parameter("max_force", max_force);

  // init wrench msg
  current_wrench.header.frame_id = "base_link";
  current_wrench.wrench.force.x = 0.0;
  current_wrench.wrench.force.y = 0.0;
  current_wrench.wrench.force.z = 0.0;
  current_wrench.wrench.torque.x = 0.0;
  current_wrench.wrench.torque.y = 0.0;
  current_wrench.wrench.torque.z = 0.0;

  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing publishers");
  _out_virtuose_status = this->create_subscription<
      raptor_api_interfaces::msg::OutVirtuoseStatus>(
      "out_virtuose_status", 1,
      std::bind(&HapticControl::out_virtuose_statusCB, this, _1));
  _out_virtuose_pose =
      this->create_subscription<raptor_api_interfaces::msg::OutVirtuosePose>(
          "out_virtuose_pose", 1,
          std::bind(&HapticControl::out_virtuose_poseCB, this, _1));

  target_pos_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_frame",
                                                              1);
  current_target_pos_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/current_frame", 1);

  _in_virtuose_force =
      this->create_publisher<raptor_api_interfaces::msg::InVirtuoseForce>(
          "in_virtuose_force", 1);
  // create force/wrench subscriber
  subscriber = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/bus0/ft_sensor0/ft_sensor_readings/wrench", 1,
      std::bind(&HapticControl::SetWrenchCB, this, _1));

  client = this->create_client<raptor_api_interfaces::srv::VirtuoseImpedance>(
      "virtuose_impedance");

  // Set a callback for this node's parameters, i.e.: "bounding_box", etc.
  cb_handle_bounding_box_ = param_subscriber_->add_parameter_callback(
      "bounding_box",
      std::bind(&HapticControl::bounding_box_CB, this, std::placeholders::_1));
  cb_scaling_factor_box_ = param_subscriber_->add_parameter_callback(
      "scaling_factor", std::bind(&HapticControl::scaling_factor_CB, this,
                                  std::placeholders::_1));
  received_haptic_pose = false;

  // Initialize the TF2 transform listener and buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // init default rotation from haptic base frame to robot base frame (-90 deg on z axis)
  Eigen::Quaterniond tmp(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  q_haptic_base_to_robot_base_.x() = tmp.x();
  q_haptic_base_to_robot_base_.y() = tmp.y();
  q_haptic_base_to_robot_base_.z() = tmp.z();
  q_haptic_base_to_robot_base_.w() = tmp.w();
}
// Dedicated function to handle parameter updates
void HapticControl::bounding_box_CB(const rclcpp::Parameter &p)
{
  RCLCPP_INFO(this->get_logger(), "Toggled BOUNDING BOX");

  use_bounding_box = (bool)p.as_int();

  bounding_box_center[0] = target_pose.pose.position.x;
  bounding_box_center[1] = target_pose.pose.position.y;
  bounding_box_center[2] = target_pose.pose.position.z;
}
void HapticControl::scaling_factor_CB(const rclcpp::Parameter &p)
{
  RCLCPP_INFO(this->get_logger(), "Received an update to the scaling factor");

  scaling_factor = std::clamp(p.as_double(), 0.0, 1.0);
}

void HapticControl::SetWrenchCB(
    const geometry_msgs::msg::WrenchStamped target_wrench)
{
  // Map forces from f/t sensor frame to base frame
  // y is x
  // x is y
  // z is -z
  current_wrench.header.stamp = target_wrench.header.stamp;
  current_wrench.wrench.force.x = target_wrench.wrench.force.y;
  current_wrench.wrench.force.y = target_wrench.wrench.force.x;
  current_wrench.wrench.force.z = -target_wrench.wrench.force.z;
  current_wrench.wrench.torque.x = target_wrench.wrench.torque.y;
  current_wrench.wrench.torque.y = target_wrench.wrench.torque.x;
  current_wrench.wrench.torque.z = -target_wrench.wrench.torque.z;
}
// Callback for topic out_virtuose_pose
void HapticControl::out_virtuose_poseCB(
    const raptor_api_interfaces::msg::OutVirtuosePose::SharedPtr msg)
{
  if (!received_haptic_pose)
  {
    // Store last pose date
    received_haptic_pose = true;
    haptic_starting_position.pose.position.x =
        msg->virtuose_pose.translation.x;
    haptic_starting_position.pose.position.y =
        msg->virtuose_pose.translation.y;
    haptic_starting_position.pose.position.z =
        msg->virtuose_pose.translation.z;
    haptic_starting_position.pose.orientation.x =
        msg->virtuose_pose.rotation.x;
    haptic_starting_position.pose.orientation.y =
        msg->virtuose_pose.rotation.y;
    haptic_starting_position.pose.orientation.z =
        msg->virtuose_pose.rotation.z;
    haptic_starting_position.pose.orientation.w =
        msg->virtuose_pose.rotation.w;
  }
  pose_date_nanosec = msg->header.stamp.nanosec;
  pose_date_sec = msg->header.stamp.sec;
  cur_pose[0] = msg->virtuose_pose.translation.x;
  cur_pose[1] = msg->virtuose_pose.translation.y;
  cur_pose[2] = msg->virtuose_pose.translation.z;
  cur_pose[3] = msg->virtuose_pose.rotation.x;
  cur_pose[4] = msg->virtuose_pose.rotation.y;
  cur_pose[5] = msg->virtuose_pose.rotation.z;
  cur_pose[6] = msg->virtuose_pose.rotation.w;
  // if (ctr % 1000 == 0)
  // {
  // printf("Virtuose pose: %f %f %f %f %f %f %f\n", cur_pose[0], cur_pose[1],
  // cur_pose[2], cur_pose[3], cur_pose[4], cur_pose[5], cur_pose[6]);
  // }
}

// Callback for topic out_virtuose_status
void HapticControl::out_virtuose_statusCB(
    const raptor_api_interfaces::msg::OutVirtuoseStatus::SharedPtr msg)
{
  // Store last status date
  status_date_nanosec = msg->header.stamp.nanosec;
  status_date_sec = msg->header.stamp.sec;
  status_state = msg->state;
  status_button = msg->buttons;
}

void HapticControl::call_impedance_service()
{
  received_ee_pose = false;
  while (!received_ee_pose)
  {
    try
    {
      auto trans = tf_buffer_->lookupTransform(base_link_name_, tool_link_name_,
                                               tf2::TimePointZero);
      ee_starting_position.pose.position.x = trans.transform.translation.x;
      ee_starting_position.pose.position.y = trans.transform.translation.y;
      ee_starting_position.pose.position.z = trans.transform.translation.z;
      ee_starting_position.pose.orientation.x = trans.transform.rotation.x;
      ee_starting_position.pose.orientation.y = trans.transform.rotation.y;
      ee_starting_position.pose.orientation.z = trans.transform.rotation.z;
      ee_starting_position.pose.orientation.w = trans.transform.rotation.w;
      received_ee_pose = true;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "End Effector pose not available: %s", ex.what());
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
  imp->base_frame.rotation.x = 0.0;
  imp->base_frame.rotation.y = 0.0;
  imp->base_frame.rotation.z = 0.0;
  imp->base_frame.rotation.w = 1.0;

  while (!client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto result = client->async_send_request(imp);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    // Store client ID given by virtuose_node
    client_id = result.get()->client_id;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Our client ID is: %d",
                client_id);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service impedance");
    return;
  }

  if (client_id == 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service impedance, client_id is zero!");
    return;
  }

  ctr = 0;

  // Perform impedance loop at 1000 Hz
  impedanceThread = this->create_wall_timer(
      1ms, std::bind(&HapticControl::ImpedanceThread, this));
  RCLCPP_INFO(this->get_logger(), "\033[0;32mImpedance thread started\033[0m");
}

void HapticControl::ImpedanceThread()
{
  if (!received_haptic_pose)
  {
    return;
  }
  // Apply the force
  raptor_api_interfaces::msg::InVirtuoseForce force;
  force.header.stamp.nanosec = get_clock()->now().nanoseconds();
  force.header.stamp.sec = get_clock()->now().seconds();
  force.client_id = client_id;
  // filter noise
  float alpha = 0;
  force.virtuose_force.force.x =
      0.3 * (alpha * old_force.virtuose_force.force.x +
             (1 - alpha) * current_wrench.wrench.force.x);
  force.virtuose_force.force.y =
      0.3 * (alpha * old_force.virtuose_force.force.y +
             (1 - alpha) * current_wrench.wrench.force.y);
  force.virtuose_force.force.z =
      0.3 * (alpha * old_force.virtuose_force.force.z +
             (1 - alpha) * current_wrench.wrench.force.z);
  // torque omitted for control simplicity
  force.virtuose_force.torque.x =
      0.0; // 0.2 * (alpha * old_force.virtuose_force.torque.x + (1 - alpha)
           // * current_wrench.wrench.torque.x);
  force.virtuose_force.torque.y =
      0.0; // 0.2 * (alpha * old_force.virtuose_force.torque.y + (1 - alpha)
           // * current_wrench.wrench.torque.y);
  force.virtuose_force.torque.z =
      0.0; // 0.2 * (alpha * old_force.virtuose_force.torque.z + (1 - alpha)
           // * current_wrench.wrench.torque.z);

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
  old_force.virtuose_force.force.x = force.virtuose_force.force.x;
  old_force.virtuose_force.force.y = force.virtuose_force.force.y;
  old_force.virtuose_force.force.z = force.virtuose_force.force.z;
  old_force.virtuose_force.torque.x = force.virtuose_force.torque.x;
  old_force.virtuose_force.torque.y = force.virtuose_force.torque.y;
  old_force.virtuose_force.torque.z = force.virtuose_force.torque.z;

  _in_virtuose_force->publish(force);
  ctr++;
  // uint64_t dt = get_clock()->now().nanoseconds() -
  //               (long long unsigned int)status_date_sec * 1000000000U +
  //               (long long unsigned int)status_date_nanosec;

  // Publish target position
  geometry_msgs::msg::PoseStamped target_pose, current_pose;
  target_pose.header.stamp.nanosec = get_clock()->now().nanoseconds();
  target_pose.header.stamp.sec = get_clock()->now().seconds();
  target_pose.header.frame_id = "base_link";

  // computing error
  Eigen::Vector3d error;
  error[0] = cur_pose[0] - haptic_starting_position.pose.position.x;
  error[1] = cur_pose[1] - haptic_starting_position.pose.position.y;
  error[2] = cur_pose[2] - haptic_starting_position.pose.position.z;

  // applying rotation from haptic base frame to robot base frame
  error = q_haptic_base_to_robot_base_.toRotationMatrix() * error;

  target_pose.pose.position.x =
      ee_starting_position.pose.position.x + error(0);
  target_pose.pose.position.y =
      ee_starting_position.pose.position.y + error(1);
  target_pose.pose.position.z =
      ee_starting_position.pose.position.z + error(2);

  // eigen quat order is w x y z
  Eigen::Quaterniond qStart(haptic_starting_position.pose.orientation.w,
                            haptic_starting_position.pose.orientation.x,
                            haptic_starting_position.pose.orientation.y,
                            haptic_starting_position.pose.orientation.z);
  Eigen::Quaterniond qCur(cur_pose[6], cur_pose[3], cur_pose[4], cur_pose[5]);
  Eigen::Quaterniond qEEStart(ee_starting_position.pose.orientation.w,
                              ee_starting_position.pose.orientation.x,
                              ee_starting_position.pose.orientation.y,
                              ee_starting_position.pose.orientation.z);

  // transform back to quaternion
  current_pose.header.frame_id = "base_link";
  current_pose.pose.position.x = target_pose.pose.position.x;
  current_pose.pose.position.y = target_pose.pose.position.y;
  current_pose.pose.position.z = target_pose.pose.position.z;

  // SAFE ZONE POSITION
  current_pose.pose.position.x = std::clamp(current_pose.pose.position.x, min_x, max_x);
  current_pose.pose.position.y = std::clamp(current_pose.pose.position.y, min_y, max_y);
  current_pose.pose.position.z = std::clamp(current_pose.pose.position.z, min_z, max_z);

  // rotation from haptic base frame to robot base frame

  // qCur = qRotB * qCur;
  // qCur.normalize();

  // // rotations to go from base frame to end effector frame
  // Eigen::Quaterniond qRotX(0.0, 1.0, 0.0, 0.0);
  // Eigen::Quaterniond qRotX(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  // Eigen::Quaterniond qRotY(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

  // Eigen::Quaterniond qRotX(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()));
  // Eigen::Quaterniond qRotY(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

  // qCur = qRotX * qCur;
  // qCur.normalize();
  // qCur = qRotY * qCur;
  // qCur.normalize();

  // qStart = qRotX * qStart;
  // qStart.normalize();
  // qStart = qRotY * qStart;
  // qStart.normalize();

  // Now qStart and qCur are expressed in the base frame,
  current_pose.pose.orientation.x = qCur.x();
  current_pose.pose.orientation.y = qCur.y();
  current_pose.pose.orientation.z = qCur.z();
  current_pose.pose.orientation.w = qCur.w();
  current_target_pos_publisher_->publish(current_pose);

  // computing the difference between the current orientation and the starting
  // orientation of the haptic device
  Eigen::Quaterniond qDiff = qCur.conjugate() * qStart;
  qDiff.normalize();

  // transform the rotation error to the robot base frame
  qDiff = q_haptic_base_to_robot_base_ * qDiff * q_haptic_base_to_robot_base_.conjugate();

  // applying delta rotation to the end effector starting orientation
  Eigen::Quaterniond qTarget = qDiff.conjugate() * qEEStart;
  qTarget.normalize();

  target_pose.pose.orientation.x = qTarget.x();
  target_pose.pose.orientation.y = qTarget.y();
  target_pose.pose.orientation.z = qTarget.z();
  target_pose.pose.orientation.w = qTarget.w();

  // publish
  target_pos_publisher_->publish(target_pose);
}

int main(int argc, char **argv)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Haptic Control node");

  rclcpp::init(argc, argv);
  auto node = std::make_shared<HapticControl>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling impedance service:");

  node->call_impedance_service();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
