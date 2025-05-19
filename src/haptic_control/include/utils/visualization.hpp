#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <vector>
#include "colors.hpp"
#include "mesh_virtual_fixtures/location.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class Visualizer {
 public:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  std::string reference_frame_;
  visualization_msgs::msg::Marker base_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;

  Visualizer(std::shared_ptr<rclcpp::Node> node,
             const std::string& reference_frame)
      : node_(node), reference_frame_(reference_frame) {
    init_publisher();
    init_base_marker(reference_frame_);
  }

  void init_base_marker(const std::string& frame_id) {
    base_marker_.header.frame_id = frame_id;
    base_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    base_marker_.action = visualization_msgs::msg::Marker::ADD;
    base_marker_.pose.orientation.w = 1.0;
    base_marker_.scale.x = base_marker_.scale.y = base_marker_.scale.z = 1.0;
    base_marker_.color.r = base_marker_.color.g = base_marker_.color.b =
        base_marker_.color.a = 1.0;
    base_marker_.mesh_use_embedded_materials = false;
  }

  void init_publisher() {
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "visualization_marker", 10);
  }

  void publish_multiple(const visualization_msgs::msg::MarkerArray& array,
                        int times = 5,
                        const std::chrono::milliseconds& interval =
                            std::chrono::milliseconds(100)) {
    for (int i = 0; i < times; ++i) {
      marker_pub_->publish(array);
      rclcpp::sleep_for(interval);
    }
  }

  void add_mesh(const std::string& path, int id) {
    visualization_msgs::msg::Marker marker = base_marker_;
    marker.header.stamp = node_->now();
    marker.id = id;
    marker.ns = "mesh";
    marker.mesh_resource = "file://" + path;
    marker_array_.markers.push_back(marker);
    publish_multiple(marker_array_);
  }

  void add_patient_mesh(const std::string& output_mesh_path,
                        const std::string& skin_mesh_path) {
    visualization_msgs::msg::Marker skin_marker = base_marker_;
    skin_marker.header.stamp = node_->now();
    skin_marker.ns = "patient";
    skin_marker.id = 0;
    skin_marker.mesh_resource = "file://" + skin_mesh_path;
    skin_marker.color.a = 0.6;

    visualization_msgs::msg::Marker vf_marker = skin_marker;
    vf_marker.id = 1;
    vf_marker.mesh_resource = "file://" + output_mesh_path;
    vf_marker.color.r = 1.0;
    vf_marker.color.g = 0.0;
    vf_marker.color.b = 0.0;
    vf_marker.color.a = 1.0;

    marker_array_.markers = {skin_marker, vf_marker};
    publish_multiple(marker_array_);
  }

  void draw_arrows(const std::vector<Eigen::Vector3d>& origins,
                   const std::vector<Eigen::Vector3d>& axis,
                   const std::vector<Eigen::Vector3d>& colors, int start_id) {
    for (size_t i = 0; i < origins.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = reference_frame_;
      marker.header.stamp = node_->now();
      marker.ns = "arrow";
      marker.id = start_id + static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point start, end;
      start.x = origins[i].x();
      start.y = origins[i].y();
      start.z = origins[i].z();
      end.x = start.x + 0.1 * axis[i].x();
      end.y = start.y + 0.1 * axis[i].y();
      end.z = start.z + 0.1 * axis[i].z();

      marker.points = {start, end};
      marker.color.r = colors[i].x();
      marker.color.g = colors[i].y();
      marker.color.b = colors[i].z();
      marker.color.a = 1.0;
      marker.scale.x = 0.01;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;

      marker_array_.markers.push_back(marker);
    }
  }

  void draw_closest_points(
      const std::unordered_map<int, std::pair<Eigen::Vector3d, Location>>&
          points,
      double radius) {
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array_.markers.push_back(clear_marker);

    visualization_msgs::msg::Marker marker = base_marker_;
    marker.header.stamp = node_->now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.ns = "CP";
    marker.scale.x = marker.scale.y = marker.scale.z = radius;

    int id = 0;
    for (const auto& [_, value] : points) {
      const auto& [pos, location] = value;
      marker.id = id++;
      marker.pose.position.x = pos.x();
      marker.pose.position.y = pos.y();
      marker.pose.position.z = pos.z();

      switch (location) {
        case Location::IN:
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          break;
        case Location::V1:
        case Location::V2:
        case Location::V3:
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          break;
        default:
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          break;
      }

      marker_array_.markers.push_back(marker);
    }
  }
  void draw_safety_sphere(const Eigen::Vector3d& center, double radius,
                          double alpha) {
    visualization_msgs::msg::Marker marker = base_marker_;
    marker.header.stamp = node_->now();
    marker.header.frame_id = reference_frame_;
    marker.ns = "safety_sphere";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z();
    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = radius * 2;
    // Blue
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = alpha;
    marker_array_.markers.push_back(marker);
  }
  void update_scene(Eigen::Vector3d target, double radius) {
    // Sphere for VF Pose
    visualization_msgs::msg::Marker marker = base_marker_;
    marker.header.stamp = node_->now();
    marker.header.frame_id = reference_frame_;
    marker.ns = "target_pose";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = target(0);
    marker.pose.position.y = target(1);
    marker.pose.position.z = target(2);
    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = radius * 2;
    // Green
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_array_.markers.push_back(marker);

    marker_pub_->publish(marker_array_);
    marker_array_.markers.clear();
  }

  void update_scene(
      const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>&
          constraint_planes,
      const Eigen::Vector3d& target, const Eigen::Vector3d& vf_pose,
      double radius, double plane_size) {
    // Sphere for VF Pose
    visualization_msgs::msg::Marker marker = base_marker_;
    marker.header.stamp = node_->now();
    marker.header.frame_id = reference_frame_;
    marker.ns = "virtual_fixture";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = vf_pose.x();
    marker.pose.position.y = vf_pose.y();
    marker.pose.position.z = vf_pose.z();
    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = radius * 2;
    // Green
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_array_.markers.push_back(marker);

    // Sphere for Target Pose
    marker.ns = "target_pose";
    marker.id = 2;
    marker.pose.position.x = target.x();
    marker.pose.position.y = target.y();
    marker.pose.position.z = target.z();
    // Red
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_array_.markers.push_back(marker);

    // Clear existing constraint planes
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.stamp = node_->now();
    clear_marker.header.frame_id = reference_frame_;
    clear_marker.ns = "constraint_planes";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array_.markers.push_back(clear_marker);

    // Add constraint planes
    Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
    for (size_t i = 0; i < constraint_planes.size(); ++i) {
      const auto& [normal, point] = constraint_planes[i];

      Eigen::Vector3d n = normal.normalized();
      Eigen::Vector3d rotation_axis = z_axis.cross(n);
      double angle = std::acos(std::clamp(z_axis.dot(n), -1.0, 1.0));
      Eigen::AngleAxisd aa =
          rotation_axis.norm() < 1e-6
              ? Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())
              : Eigen::AngleAxisd(angle, rotation_axis.normalized());
      Eigen::Quaterniond q(aa);
      q.normalize();
      marker = base_marker_;
      marker.header.stamp = node_->now();
      marker.header.frame_id = reference_frame_;
      marker.ns = "constraint_planes";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = plane_size;
      marker.scale.y = plane_size;
      marker.scale.z = 0.0001;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.5;
      marker.pose.position.x = point.x();
      marker.pose.position.y = point.y();
      marker.pose.position.z = point.z();
      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();

      marker_array_.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array_);
    marker_array_.markers.clear();
  }

  void draw_cone(const Eigen::Vector3d& position,
                 const Eigen::Quaterniond& orientation, const double angle,
                 const int id = 0,
                 const std::vector<double> rgba = {1.0, 0.0, 0.0, 1.0}) {
    visualization_msgs::msg::Marker marker = base_marker_;
    marker.header.stamp = node_->now();
    marker.ns = "cone";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.color.r = rgba[0];
    marker.color.g = rgba[1];
    marker.color.b = rgba[2];
    marker.color.a = 0.2;  // rgba[3];
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();

    static const int NUM_SEGMENTS = 32;  // more segments = smoother cone
    const double height = 0.1;           // cone height
    const double radius = height * std::tan(angle);  // base radius

    // Create cone tip point
    geometry_msgs::msg::Point tip;
    tip.x = 0.0;
    tip.y = 0.0;
    tip.z = 0.0;  // tip at local (0,0,0)

    double delta_theta = 2.0 * M_PI / NUM_SEGMENTS;

    // Build triangles: from tip to base circle
    for (int i = 0; i < NUM_SEGMENTS; ++i) {
      double theta = i * delta_theta;
      double next_theta = (i + 1) * delta_theta;

      geometry_msgs::msg::Point p1, p2;

      // First point on the base circle
      p1.x = radius * std::cos(theta);
      p1.y = radius * std::sin(theta);
      p1.z = height;

      // Next point on the base circle
      p2.x = radius * std::cos(next_theta);
      p2.y = radius * std::sin(next_theta);
      p2.z = height;

      // Triangle: tip, p1, p2
      marker.points.push_back(tip);
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }

    marker_array_.markers.push_back(marker);
  }
};

#endif  // VISUALIZATION_HPP
