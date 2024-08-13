#include "vf_test.hpp"

using namespace Eigen;

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

VirtualFixtureTest::VirtualFixtureTest(const std::string &name,
                                             const std::string &namespace_,
                                             const rclcpp::NodeOptions &options)
    : Node(name, namespace_, options)
{
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_frame", 1, std::bind(&VirtualFixtureTest::PoseCB, this, _1));
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "visualization_marker", 1);
  load_path_ = "/root/SKEL_WS/ros2_ws/projected_skel.ply";
  // vf_calculator_ = std::make_shared<VirtualFixtureCalculator>(load_path_);
  sphere_radius_ = 0.02;
  ClearRviz();
  UpdateRibCageRviz();
  RCLCPP_INFO(this->get_logger(), "Virtual Fixture Test Node Initialized");
}
void VirtualFixtureTest::PoseCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received target frame: %f %f %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  UpdateRviz(*msg, sphere_radius_);  
  Eigen::Vector3d target_position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
   vf_calculator_->EnforceVirtualFixture(target_position, sphere_radius_);

}
void VirtualFixtureTest::UpdateRviz(geometry_msgs::msg::PoseStamped pose, double radius)
{
  // Update the sphere and reference frame
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = this->now();
  marker.id = 1; // sphere target 
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose.pose;
  marker.scale.x = radius;
  marker.scale.y = radius;
  marker.scale.z = radius;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  
  marker.color.a = 1.0;
  marker_pub_->publish(marker);
  
}
void VirtualFixtureTest::ClearRviz(){
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_pub_->publish(marker);
}

void VirtualFixtureTest::UpdateRibCageRviz()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();

    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD; 
    marker.mesh_resource = "file://"+load_path_;
    marker.mesh_use_embedded_materials = true;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    RCLCPP_INFO(this->get_logger(), "Rib cage mesh loaded");
    marker_pub_->publish(marker);
}