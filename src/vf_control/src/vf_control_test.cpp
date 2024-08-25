#include "vf_control.hpp"

#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;

VFControl::VFControl(const std::string &name, const std::string &namespace_,
                     const rclcpp::NodeOptions &options)
    : Node(name, namespace_, options) {
  target_pos_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/target_frame_vf", 1);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "visualization_marker", 1);

  x_new_ << 0.0, 0.0, 0.3;
  vf_pose_ << 0.0, 0.0, 0.3;

  // Initializes the TF2 transform listener and buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // open3d::data::KnotMesh dataset;
  // auto o3d_mesh = open3d::io::CreateMeshFromFile(dataset.GetPath());
  // o3d_mesh->Scale(0.002, Eigen::Vector3d(0, 0, 0));
  auto o3d_mesh = open3d::geometry::TriangleMesh::CreateSphere(0.2,40);
  open3d::io::WriteTriangleMesh(load_path_, *o3d_mesh);

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Loaded mesh with "
                         << o3d_mesh->vertices_.size() << " vertices and "
                         << o3d_mesh->triangles_.size() << " triangles.");
  if (o3d_mesh->vertices_.empty()) {
    std::cerr << "The loaded mesh contains no vertices." << std::endl;
    rclcpp::shutdown();
  }
  o3d_mesh->ComputeTriangleNormals();
  o3d_mesh->OrientTriangles();
  o3d_mesh->NormalizeNormals();

  auto point_cloud = std::make_shared<open3d::geometry::PointCloud>();
  point_cloud->points_ = o3d_mesh->vertices_;

  RCLCPP_INFO_STREAM(this->get_logger(), "Computing mesh properties...");

  mesh_ = std::make_shared<Mesh>(o3d_mesh->vertices_, o3d_mesh->triangles_,
                                 o3d_mesh->triangle_normals_);
  rclcpp::sleep_for(1s);  // idk why it is needed
  AddMesh();
  visualizationThread_ =
      this->create_wall_timer(10ms, std::bind(&VFControl::UpdateScene, this));

  // Start the keyboard input thread
  std::thread input_thread(&VFControl::KeyboardInputLoop, this);
  input_thread.detach();

  RCLCPP_INFO(this->get_logger(),
              "\033[0;32mVisualization thread started\033[0m");
}

// Function to handle keyboard input
void VFControl::KeyboardInputLoop() {
  char c;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  Eigen::Vector3d direction(0, 0, 0);
  while (rclcpp::ok()) {
    c = getchar();

    // WASD + Arrow keys control
    if (c == 'w') direction(1) += 1;
    if (c == 's') direction(1) -= 1;
    if (c == 'a') direction(0) -= 1;
    if (c == 'd') direction(0) += 1;
    if (c == 'q') direction(2) += 1;
    if (c == 'e') direction(2) -= 1;
    x_new_ = x_new_ + direction * 0.003;
    ResetPlanes();
    Eigen::Vector3d delta_x = enforce_virtual_fixture(
        *mesh_, x_new_, vf_pose_, radius_, constraint_planes_);
    vf_pose_ += 0.9 * delta_x;
    direction << 0.0, 0.0, 0.0;
  }

  // Restore the old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

void VFControl::AddMesh() {
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = base_link_name_;
  marker.header.stamp = this->now();
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.ns = "rib_cage";
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.mesh_resource = "file://" + load_path_;
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
  marker_array.markers.push_back(marker);
  for (int i = 0; i < 20; i++) {
    marker_pub_->publish(marker_array);
    rclcpp::sleep_for(100ms);
  }
}

void VFControl::ResetPlanes() {
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = base_link_name_;
  marker.header.stamp = get_clock()->now();
  marker.ns = "constraint_planes";
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(marker);
  marker_pub_->publish(marker_array);
  constraint_planes_.clear();
}

void VFControl::UpdateScene() {
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = base_link_name_;
  target_pose.header.stamp = get_clock()->now();
  target_pose.pose.position.x = x_new_(0);
  target_pose.pose.position.y = x_new_(1);
  target_pose.pose.position.z = x_new_(2);
  target_pos_publisher_->publish(target_pose);

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = base_link_name_;
  marker.header.stamp = get_clock()->now();
  marker.id = 1;
  marker.ns = "virtual_fixture";
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = vf_pose_(0);
  marker.pose.position.y = vf_pose_(1);
  marker.pose.position.z = vf_pose_(2);
  marker.scale.x = radius_ * 2;
  marker.scale.y = radius_ * 2;
  marker.scale.z = radius_ * 2;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker_array.markers.push_back(marker);

  marker.ns = "target_pose";
  marker.id = 2;
  marker.pose.position.x = x_new_(0);
  marker.pose.position.y = x_new_(1);
  marker.pose.position.z = x_new_(2);
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker_array.markers.push_back(marker);

  marker.ns = "constraint_planes";
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.0001;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
  for (size_t i = 0; i < constraint_planes_.size(); i++) {
    auto n = constraint_planes_[i].first;
    n = n.normalized();
    auto p = constraint_planes_[i].second;
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

int main(int argc, char **argv) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting VF Control node");

  rclcpp::init(argc, argv);
  auto node = std::make_shared<VFControl>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling impedance service:");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}