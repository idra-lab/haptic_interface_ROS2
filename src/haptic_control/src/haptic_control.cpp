#include <cstdio>
#include <memory>
#include <chrono>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_status.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_pose.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_physical_pose.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_speed.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_force.hpp"
#include "raptor_api_interfaces/msg/in_virtuose_pose.hpp"
#include "raptor_api_interfaces/msg/in_virtuose_speed.hpp"
#include "raptor_api_interfaces/msg/in_virtuose_force.hpp"
#include "raptor_api_interfaces/srv/virtuose_impedance.hpp"
#include "raptor_api_interfaces/srv/virtuose_reset.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/wait_for_message.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;




using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class HapticControl : public rclcpp::Node
{
public:
  HapticControl(
      const std::string &name = "haptic_control",
      const std::string &namespace_ = "",
      const rclcpp::NodeOptions &options = (rclcpp::NodeOptions()
                                                .allow_undeclared_parameters(true)
                                                .automatically_declare_parameters_from_overrides(true)))
      : Node(name, namespace_, options)
  {
    

    this->get_parameter("horizontal_table_z", horizontal_table_z);
    this->get_parameter("max_force", max_force);
    this->get_parameter("penalty_stiffness", penalty_stiffness);

    // init wrench msg
    current_wrench.header.frame_id = "base_link";
    current_wrench.wrench.force.x = 0.0;
    current_wrench.wrench.force.y = 0.0;
    current_wrench.wrench.force.z = 0.0;
    current_wrench.wrench.torque.x = 0.0;
    current_wrench.wrench.torque.y = 0.0;
    current_wrench.wrench.torque.z = 0.0;

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing publishers");
    _out_virtuose_status = this->create_subscription<raptor_api_interfaces::msg::OutVirtuoseStatus>("out_virtuose_status", 1, std::bind(&HapticControl::out_virtuose_statusCB, this, _1));
    _out_virtuose_pose = this->create_subscription<raptor_api_interfaces::msg::OutVirtuosePose>("out_virtuose_pose", 1, std::bind(&HapticControl::out_virtuose_poseCB, this, _1));

    _current_ee_pos = this->create_subscription<geometry_msgs::msg::PoseStamped>("/tool0_pose", 1, std::bind(&HapticControl::current_ee_posCB, this, _1));
    _target_pos_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_frame", 1);
    current_target_pos_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_frame", 1);
   


    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing subscriptions");
    _in_virtuose_force = this->create_publisher<raptor_api_interfaces::msg::InVirtuoseForce>("in_virtuose_force", 1);
    // create force/wrench subscriber
    subscriber = this->create_subscription<geometry_msgs::msg::WrenchStamped>("/bus0/ft_sensor0/ft_sensor_readings/wrench", 1, std::bind(&HapticControl::SetWrenchCB, this, _1));
    // subscriber = this->create_subscription<geometry_msgs::msg::WrenchStamped>("/ft_sensor_wrench", 1, std::bind(&HapticControl::SetWrenchCB, this, _1));


    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing services");
    client = this->create_client<raptor_api_interfaces::srv::VirtuoseImpedance>("virtuose_impedance");

    received_haptic_pose = false;
  }

// private:
  void current_ee_posCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    ee_current_pose.pose.position.x = msg->pose.position.x;
    ee_current_pose.pose.position.y = msg->pose.position.y;
    ee_current_pose.pose.position.z = msg->pose.position.z;
    ee_current_pose.pose.orientation.x = msg->pose.orientation.x;
    ee_current_pose.pose.orientation.y = msg->pose.orientation.y;
    ee_current_pose.pose.orientation.z = msg->pose.orientation.z;
    ee_current_pose.pose.orientation.w = msg->pose.orientation.w;
  }

  void SetWrenchCB(const geometry_msgs::msg::WrenchStamped target_wrench)
  {
    // y is x 
    // x is y
    // z is -z
    current_wrench.header.stamp = target_wrench.header.stamp;
    current_wrench.wrench.force.x = target_wrench.wrench.force.y;
    current_wrench.wrench.force.y = target_wrench.wrench.force.x;
    current_wrench.wrench.force.z = - target_wrench.wrench.force.z;
    current_wrench.wrench.torque.x = target_wrench.wrench.torque.y;
    current_wrench.wrench.torque.y = target_wrench.wrench.torque.x;
    current_wrench.wrench.torque.z = - target_wrench.wrench.torque.z;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current force is: %f", current_wrench.wrench.force.z);
  }
  // Callback for topic out_virtuose_pose
  void out_virtuose_poseCB(const raptor_api_interfaces::msg::OutVirtuosePose::SharedPtr msg)
  {
    if(!received_haptic_pose){
    // Store last pose date
      received_haptic_pose = true;
      haptic_starting_position.pose.position.x = msg->virtuose_pose.translation.x;
      haptic_starting_position.pose.position.y = msg->virtuose_pose.translation.y;
      haptic_starting_position.pose.position.z = msg->virtuose_pose.translation.z;
      haptic_starting_position.pose.orientation.x = msg->virtuose_pose.rotation.x;
      haptic_starting_position.pose.orientation.y = msg->virtuose_pose.rotation.y;
      haptic_starting_position.pose.orientation.z = msg->virtuose_pose.rotation.z;
      haptic_starting_position.pose.orientation.w = msg->virtuose_pose.rotation.w;
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
    // printf("Virtuose pose: %f %f %f %f %f %f %f\n", cur_pose[0], cur_pose[1], cur_pose[2], cur_pose[3], cur_pose[4], cur_pose[5], cur_pose[6]);
    // }
  }

  // Callback for topic out_virtuose_status
  void out_virtuose_statusCB(const raptor_api_interfaces::msg::OutVirtuoseStatus::SharedPtr msg)
  {
    // Store last status date
    status_date_nanosec = msg->header.stamp.nanosec;
    status_date_sec = msg->header.stamp.sec;
    status_state = msg->state;
    status_button = msg->buttons;
  }

  void call_impedance_service()
  {

    received_ee_pose = false;
    while (!received_ee_pose)
    {
      received_ee_pose = rclcpp::wait_for_message(ee_starting_position, this->shared_from_this(), "/tool0_pose", std::chrono::seconds(1));
      RCLCPP_INFO(this->get_logger(), "still waiting for ee position: result is %d" , received_ee_pose);
    }
    RCLCPP_INFO(this->get_logger(), "\n\nEnd Effector Pose received \nEE starting position is : x: %f | y: %f | z: %f | \nEE starting rotation is : x: %f | y: %f | z: %f | w: %f\n",
                ee_starting_position.pose.position.x, ee_starting_position.pose.position.y, ee_starting_position.pose.position.z,
                ee_starting_position.pose.orientation.x, ee_starting_position.pose.orientation.y, ee_starting_position.pose.orientation.z, ee_starting_position.pose.orientation.w
                );


    // Request impedance mode

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending impedance request");

    auto imp = std::make_shared<raptor_api_interfaces::srv::VirtuoseImpedance::Request>();
    this->get_parameter("channel", imp->channel);
    this->get_parameter("ff_device_ip_address", imp->ff_device_ip_address);
    this->get_parameter("ff_device_param_file", imp->ff_device_param_file);
    this->get_parameter("local_ip_address", imp->local_ip_address);
    this->get_parameter("speed_factor", imp->speed_factor);
    this->get_parameter("force_factor", imp->force_factor);
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
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(imp);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      // Store client ID given by virtuose_node
      client_id = result.get()->client_id;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Our client ID is: %d", client_id);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service impedance");
      return;
    }

    if (client_id == 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service impedance, client_id is zero!");
      return;
    }

    ctr = 0;

    // Wait for first pose of device and fill starting pose

    // while (!received_haptic_pose)
    // {
    //   received_haptic_pose = rclcpp::wait_for_message(haptic_starting_position, this->shared_from_this(), "/out_virtuose_pose", std::chrono::seconds(1));
    //   RCLCPP_INFO(this->get_logger(), "still waiting for haptic device position");
    // }
    // RCLCPP_INFO(this->get_logger(), "Haptic Device pose received \nHaptic starting position is : x: %f | y: %f | z: %f",
    //             haptic_starting_position.pose.position.x, haptic_starting_position.pose.position.y, haptic_starting_position.pose.position.z);

    // Perform impedance loop at 1000 Hz
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Before");

    impedanceThread = this->create_wall_timer(1ms, std::bind(&HapticControl::ImpedanceThread, this));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "After");
  }

  void ImpedanceThread()
  {
    if(!received_haptic_pose){
      return;
    }
    // Apply the force
    raptor_api_interfaces::msg::InVirtuoseForce force;
    force.header.stamp.nanosec = get_clock()->now().nanoseconds();
    force.header.stamp.sec = get_clock()->now().seconds();
    force.client_id = client_id;
    // filter noise
    float alpha = 0;
    force.virtuose_force.force.x = 0.3 *  (alpha * old_force.virtuose_force.force.x + (1 - alpha) * current_wrench.wrench.force.x);
    force.virtuose_force.force.y = 0.3 *  (alpha * old_force.virtuose_force.force.y + (1 - alpha) * current_wrench.wrench.force.y);
    force.virtuose_force.force.z = 0.3 *  (alpha * old_force.virtuose_force.force.z + (1 - alpha) * current_wrench.wrench.force.z);
    force.virtuose_force.torque.x = 0.0; //0.2 * (alpha * old_force.virtuose_force.torque.x + (1 - alpha) * current_wrench.wrench.torque.x);
    force.virtuose_force.torque.y = 0.0; //0.2 * (alpha * old_force.virtuose_force.torque.y + (1 - alpha) * current_wrench.wrench.torque.y);
    force.virtuose_force.torque.z = 0.0; //0.2 * (alpha * old_force.virtuose_force.torque.z + (1 - alpha) * current_wrench.wrench.torque.z);

    // SAFE ZONE FORCE

    force.virtuose_force.force.x = std::clamp(force.virtuose_force.force.x,-6.0,6.0);
    force.virtuose_force.force.y = std::clamp(force.virtuose_force.force.y,-6.0,6.0);
    force.virtuose_force.force.z = std::clamp(force.virtuose_force.force.z,-6.0,6.0);
    
    // // SAFE ZONE TORQUE
    // force.virtuose_force.torque.x = std::clamp(force.virtuose_force.torque.x,-0.2,0.2);
    // force.virtuose_force.torque.y = std::clamp(force.virtuose_force.torque.y,-0.2,0.2);
    // force.virtuose_force.torque.z = std::clamp(force.virtuose_force.torque.z,-0.2,0.2);
    
    
    // updating old force
    old_force.virtuose_force.force.x = force.virtuose_force.force.x;
    old_force.virtuose_force.force.y = force.virtuose_force.force.y;
    old_force.virtuose_force.force.z = force.virtuose_force.force.z;
    old_force.virtuose_force.torque.x = force.virtuose_force.torque.x;
    old_force.virtuose_force.torque.y = force.virtuose_force.torque.y;
    old_force.virtuose_force.torque.z = force.virtuose_force.torque.z;



    _in_virtuose_force->publish(force);
    ctr++;
    uint64_t dt = get_clock()->now().nanoseconds() - (long long unsigned int)status_date_sec * 1000000000U + (long long unsigned int)status_date_nanosec;
    




    // Publish target position
    geometry_msgs::msg::PoseStamped target_pose, current_pose;
    target_pose.header.stamp.nanosec = get_clock()->now().nanoseconds();
    target_pose.header.stamp.sec = get_clock()->now().seconds();
    target_pose.header.frame_id = "base_link";

    target_pose.pose.position.x = ee_starting_position.pose.position.x + (cur_pose[0] - haptic_starting_position.pose.position.x);
    target_pose.pose.position.y = ee_starting_position.pose.position.y + (cur_pose[1] - haptic_starting_position.pose.position.y);
    target_pose.pose.position.z = ee_starting_position.pose.position.z + (cur_pose[2] - haptic_starting_position.pose.position.z);

    //eigen quat order is w x y z
    Eigen::Quaterniond qStart(haptic_starting_position.pose.orientation.w, haptic_starting_position.pose.orientation.x, haptic_starting_position.pose.orientation.y, haptic_starting_position.pose.orientation.z);
    Eigen::Quaterniond qCur(cur_pose[6], cur_pose[3], cur_pose[4], cur_pose[5]);
    Eigen::Quaterniond qEEStart(ee_starting_position.pose.orientation.w, ee_starting_position.pose.orientation.x, ee_starting_position.pose.orientation.y, ee_starting_position.pose.orientation.z);

    // // transform to rotation matrix
    // Eigen::Matrix3d rStart = qStart.toRotationMatrix();
    // Eigen::Matrix3d rCur = qCur.toRotationMatrix();
    // Eigen::Matrix3d rEEStart = qEEStart.toRotationMatrix();

    // // compute rotation matrix difference
    // Eigen::Matrix3d rDiff =  rCur * rStart.transpose() ;

    // // apply rotation matrix difference to target rotation matrix
    // Eigen::Matrix3d rTarget = rEEStart * rDiff;

    // transform back to quaternion
    current_pose.header.frame_id = "base_link";
    current_pose.pose.position.x =  target_pose.pose.position.x;
    current_pose.pose.position.y =  target_pose.pose.position.y;
    current_pose.pose.position.z =  target_pose.pose.position.z;


    Eigen::Quaterniond qRotX(0.0, 1.0, 0.0, 0.0);
    Eigen::Quaterniond qRotY(0.70710678118, 0.0, 0, 0.70710678118);
    
    qCur = qRotX * qCur;
    qCur.normalize();
    qCur = qRotY * qCur;
    qCur.normalize(); 

    qStart =  qRotX * qStart;
    qStart.normalize();
    qStart =  qRotY * qStart;
    qStart.normalize();

    current_pose.pose.orientation.x =  qCur.x();
    current_pose.pose.orientation.y =  qCur.y();
    current_pose.pose.orientation.z =  qCur.z();
    current_pose.pose.orientation.w =  qCur.w();
    current_target_pos_publisher->publish(current_pose);

    Eigen::Quaterniond qDiff = qCur.conjugate() * qStart;
    qDiff.normalize();

    Eigen::Quaterniond qTarget = qDiff.conjugate() * qEEStart ; 
    qTarget.normalize();

    // qTarget = Eigen::Quaterniond(0.0, 1, 0.0, 0.0) * qTarget; 
    // qTarget.normalize();

    target_pose.pose.orientation.x = qTarget.x();
    target_pose.pose.orientation.y = qTarget.y();
    target_pose.pose.orientation.z = qTarget.z();
    target_pose.pose.orientation.w = qTarget.w();

    // publish
    _target_pos_publisher->publish(target_pose);
  
    
    
    // Print status every second
    if (ctr % 100 == 0)
    {
      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "start x : %f | %f | %f \n", ee_starting_position.pose.position.x, haptic_starting_position.pose.position.x, cur_pose[0]);
      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Status: %llu %d %d %f %f %f", dt, status_state, status_button,
      //        cur_pose[0], cur_pose[1], cur_pose[2]);
      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x: %f | y: %f | z: %f, | Force_z: %f ", cur_pose[0], cur_pose[1], cur_pose[2], current_wrench.wrench.force.z);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "START x: %f | y: %f | z: %f, ", haptic_starting_position.pose.position.x, haptic_starting_position.pose.position.y, haptic_starting_position.pose.position.z);
      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DELTA x: %f | y: %f | z: %f, | w: %f ", qDiff.x(), qDiff.y(), qDiff.z(), qDiff.w());
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TARGET x: %f | y: %f | z: %f, | w: %f ", qTarget.x(), qTarget.y(), qTarget.z(), qTarget.w());
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "###############################################");


    }


  }

  // Storage for virtuose pose
  float cur_pose[7];
  uint32_t pose_date_nanosec;
  uint32_t pose_date_sec;
  int ctr = 0;

  // Storage for virtuose_node status
  int status_state = 0;
  int status_button;
  uint32_t client_id = 0;

  uint32_t status_date_nanosec;
  uint32_t status_date_sec;
  uint32_t start;
  geometry_msgs::msg::WrenchStamped current_wrench;

  float penalty_stiffness, horizontal_table_z, max_force;

  rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuoseStatus>::SharedPtr _out_virtuose_status;
  rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuosePose>::SharedPtr _out_virtuose_pose;
  rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuoseForce>::SharedPtr _out_virtuose_force;
  rclcpp::Publisher<raptor_api_interfaces::msg::InVirtuoseForce>::SharedPtr _in_virtuose_force;
  rclcpp::Client<raptor_api_interfaces::srv::VirtuoseImpedance>::SharedPtr client;
  raptor_api_interfaces::msg::InVirtuoseForce old_force;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber;

  // position control
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _current_ee_pos;
  rclcpp ::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pos_publisher;
  rclcpp ::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_target_pos_publisher;

  geometry_msgs::msg::PoseStamped ee_current_pose;
  geometry_msgs::msg::PoseStamped ee_starting_position;
  geometry_msgs::msg::PoseStamped haptic_starting_position;
  bool received_ee_pose, received_haptic_pose;





  rclcpp::TimerBase::SharedPtr impedanceThread;
};
// Main function
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
