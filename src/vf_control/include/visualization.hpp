#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "location.hpp"

using namespace std::chrono_literals;
class Visualizer
{
public:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  std::string reference_frame_;
  double plane_size_;

  void InitMsg(visualization_msgs::msg::Marker &marker)
  {
    marker.header.frame_id = reference_frame_;
    marker.header.stamp = node_->now();
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.mesh_use_embedded_materials = true;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0;
  }
  void AddMesh(std::string path, int id)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    InitMsg(marker);
    marker.id = id;
    marker.ns = "mesh";
    marker.mesh_resource = "file://" + path;
    marker.color.a = 1.0;
    marker_array.markers.push_back(marker);
    for (int i = 0; i < 20; i++)
    {
      marker_pub_->publish(marker_array);
      rclcpp::sleep_for(100ms);
    }
  }
  void AddPatientMesh(std::string output_mesh_path,
                      std::string skin_mesh_path)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    InitMsg(marker);
    marker.ns = "patient";
    // add skin mesh
    marker.id = 0;
    marker.color.a = 0.2;
    marker.mesh_resource = "file://" + skin_mesh_path;
    marker_array.markers.push_back(marker);
    // add vf
    marker.mesh_resource = "file://" + output_mesh_path;
    marker.id = 1;
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.g = 0.0;
    marker_array.markers.push_back(marker);
    for (int i = 0; i < 20; i++)
    {
      marker_pub_->publish(marker_array);
      rclcpp::sleep_for(100ms);
    }
    RCLCPP_INFO(node_->get_logger(), "Mesh visualization completed");
  }
  void DrawClosestPoints(std::unordered_map<int, std::pair<Eigen::Vector3d, Location>> &points, double radius)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    InitMsg(marker);
    marker.ns = "CP";
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.color.a = 1.0;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;
    // iterating over the map
    int it = 0;
    for (auto [i, point] : points)
    {
      marker.id = it++;
      if (point.second == Location::IN)
      {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      }
      else if (point.second == Location::V1 || point.second == Location::V2 || point.second == Location::V3)
      {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
      }
      else
      {
        // on edge
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
      }
      marker.pose.position.x = point.first(0);
      marker.pose.position.y = point.first(1);
      marker.pose.position.z = point.first(2);
      marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
  }

  void UpdateScene(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
                       constraint_planes,
                   Eigen::Vector3d target, Eigen::Vector3d vf_pose,
                   double radius)
  {

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = reference_frame_;
    marker.header.stamp = node_->now();
    marker.id = 1;
    marker.ns = "virtual_fixture";
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = vf_pose(0);
    marker.pose.position.y = vf_pose(1);
    marker.pose.position.z = vf_pose(2);
    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = radius * 2;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);

    marker.ns = "target_pose";
    marker.id = 2;
    marker.pose.position.x = target(0);
    marker.pose.position.y = target(1);
    marker.pose.position.z = target(2);
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker_array.markers.push_back(marker);

    marker.ns = "constraint_planes";
    // reset planes
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = plane_size_;
    marker.scale.y = plane_size_;
    marker.scale.z = 0.0001;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
    for (size_t i = 0; i < constraint_planes.size(); i++)
    {
      auto n = constraint_planes[i].first;
      n = n.normalized();
      auto p = constraint_planes[i].second;
      auto rotation_axis = z_axis.cross(n);
      auto rotation_angle = std::acos(z_axis.dot(n));
      Eigen::Quaterniond q;
      q = Eigen::AngleAxisd(rotation_angle, rotation_axis);
      marker.id = i;
      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();
      marker.pose.position.x = p(0);
      marker.pose.position.y = p(1);
      marker.pose.position.z = p(2);
      marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
  }
  Visualizer(std::shared_ptr<rclcpp::Node> node, std::string reference_frame, double plane_size)
      : node_(node), reference_frame_(reference_frame), plane_size_(plane_size)
  {
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "visualization_marker", 1);
  }
};

#endif // VISUALIZATION_HPP