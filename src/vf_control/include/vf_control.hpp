#ifndef VF_CONTROL_HPP
#define VF_CONTROL_HPP

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Dense>
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
#include "raptor_api_interfaces/srv/virtuose_impedance.hpp"
#include "raptor_api_interfaces/srv/virtuose_reset.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
// my includes
#include "closest_on_triangle.hpp"
#include "mesh.hpp"
#include "vf_computation.hpp"
#include "visualization.hpp"
#include <omp.h>

class VFControl : public rclcpp::Node
{
public:
    VFControl(const std::string &name = "vf_control",
              const std::string &namespace_ = "",
              const rclcpp::NodeOptions &options =
                  rclcpp::NodeOptions()
                      .allow_undeclared_parameters(true)
                      .automatically_declare_parameters_from_overrides(true));
    // void UpdateRviz(geometry_msgs::msg::PoseStamped pose, double radius,
    // std::vector<double> color, uint id); void ClearRviz(); void
    // UpdateRibCageRviz();
    void KeyboardInputLoop();

    void SetWrenchCB(const geometry_msgs::msg::WrenchStamped target_wrench);
    void out_virtuose_pose_CB(
        const raptor_api_interfaces::msg::OutVirtuosePose::SharedPtr msg);
    void out_virtuose_statusCB(
        const raptor_api_interfaces::msg::OutVirtuoseStatus::SharedPtr msg);
    void bag_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void call_impedance_service();
    void impedanceThread();
    void Initialize();

private:
    // ROS2 subscribtions
    rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuoseStatus>::SharedPtr
        out_virtuose_status_;
    rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuosePose>::SharedPtr
        _out_virtuose_pose_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
        ft_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        bag_subscriber_;
    // ROS2 publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        target_pos_publisher_;
    rclcpp::Publisher<raptor_api_interfaces::msg::InVirtuoseForce>::SharedPtr
        _in_virtuose_force;
    // ROS2 service client
    rclcpp::Client<raptor_api_interfaces::srv::VirtuoseImpedance>::SharedPtr
        client_;
    // ROS2 timers
    rclcpp::TimerBase::SharedPtr pose_update_timer_;
    rclcpp::TimerBase::SharedPtr impedanceThread_;

    // MESH
    std::shared_ptr<Mesh> mesh_;
    // Visualizer visualizer_;  
    Eigen::Vector3d x_new_, vf_pose_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> constraint_planes_;
    // VARIABLES
    std::string base_link_name_ = "world";
    std::string input_mesh_path_, output_mesh_path_, skin_mesh_path_, mesh_type_;
    std::shared_ptr<Visualizer> visualizer_;
    double plane_size_;
    int client__id_;
    int ctr_;
    double radius_, lookup_area_;
};

#endif // VF_CONTROL_HPP