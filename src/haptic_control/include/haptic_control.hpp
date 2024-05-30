#ifndef HAPTIC_CONTROL_HPP
#define HAPTIC_CONTROL_HPP

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
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
#include "raptor_api_interfaces/srv/virtuose_impedance.hpp"
#include "raptor_api_interfaces/srv/virtuose_reset.hpp"

using namespace Eigen;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class HapticControl : public rclcpp::Node
{
public:
    HapticControl(const std::string &name = "haptic_control",
                  const std::string &namespace_ = "",
                  const rclcpp::NodeOptions &options =
                      rclcpp::NodeOptions()
                          .allow_undeclared_parameters(true)
                          .automatically_declare_parameters_from_overrides(true));
    void bounding_box_CB(const rclcpp::Parameter &p);
    void scaling_factor__CB(const rclcpp::Parameter &p);
    void update_current_ee_pos();
    void SetWrenchCB(const geometry_msgs::msg::WrenchStamped target_wrench);
    void out_virtuose_pose_CB(
        const raptor_api_interfaces::msg::OutVirtuosePose::SharedPtr msg);
    void out_virtuose_statusCB(
        const raptor_api_interfaces::msg::OutVirtuoseStatus::SharedPtr msg);
    void call_impedance_service();
    void impedanceThread();

private:
    // ROS2 subscribtions
    rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuoseStatus>::SharedPtr
        out_virtuose_status_;
    rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuosePose>::SharedPtr
        _out_virtuose_pose_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_subscriber_;
    // ROS2 publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        target_pos_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        current_target_pos_publisher_;
    rclcpp::Publisher<raptor_api_interfaces::msg::InVirtuoseForce>::SharedPtr
        _in_virtuose_force;
    // ROS2 service client_s
    rclcpp::Client<raptor_api_interfaces::srv::VirtuoseImpedance>::SharedPtr
        client_;
    // ROS2 timers
    rclcpp::TimerBase::SharedPtr pose_update_timer_;
    rclcpp::TimerBase::SharedPtr impedanceThread_;
    // ROS2 tf2 transform listener and buffer
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // ROS2 params callbacks
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_bounding_box_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_scaling_factor__box_;

    geometry_msgs::msg::WrenchStamped current_wrench_;
    geometry_msgs::msg::PoseStamped target_pose_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped ee_starting_position;
    geometry_msgs::msg::PoseStamped ee_current_pose_;
    geometry_msgs::msg::PoseStamped haptic_starting_position_;
    Eigen::Quaterniond q_haptic_base_to_robot_base_;
    raptor_api_interfaces::msg::InVirtuoseForce old_force_;
    double scaling_factor_;
    double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
    double max_force_;
    double force_scale_;
    std::string tool_link_name_;
    std::string base_link_name_;
    std::string ft_link_name_;
    bool use_bounding_box_;
    bool received_haptic_pose_;
    bool received_ee_pose_;
    bool use_limits;
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
};

#endif // HAPTIC_CONTROL_HPP
