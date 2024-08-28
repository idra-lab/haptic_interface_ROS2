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
    : Node(name, namespace_, options)
{
  target_pos_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("target_frame_vf",
                                                              1);

  this->mesh_type_ = this->get_parameter("mesh_type").as_string();
  this->input_mesh_path_ = this->get_parameter("input_mesh_path").as_string();
  this->output_mesh_path_ = this->get_parameter("output_mesh_path").as_string();
  this->radius_ = this->get_parameter("radius").as_double();
  this->lookup_area_ = this->get_parameter("lookup_area").as_double();
  this->plane_size_ = this->get_parameter("plane_size").as_double();

  // Initializes the TF2 transform listener and buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}
void VFControl::Initialize()
{
  x_new_ << -0.0, 0.0, 0.2;
  vf_pose_ << -0.0, 0.0, 0.2;

  auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
  visualizer_ = std::make_shared<Visualizer>(this->shared_from_this(), base_link_name_, plane_size_);

  if (mesh_type_ == "bunny")
  {
    open3d::data::BunnyMesh dataset;
    o3d_mesh = open3d::io::CreateMeshFromFile(dataset.GetPath());
  }
  else if (mesh_type_ == "knot")
  {
    open3d::data::KnotMesh dataset;
    o3d_mesh = open3d::io::CreateMeshFromFile(dataset.GetPath());
    o3d_mesh->Scale(0.002, Eigen::Vector3d(0, 0, 0));
  }
  else if (mesh_type_ == "sphere")
  {
    x_new_ << -0.09, -0.0024, 0.20140033;
    vf_pose_ << -0.09, -0.0024, 0.20140033;
    o3d_mesh = open3d::geometry::TriangleMesh::CreateSphere(0.2, 20);
  }
  else if (mesh_type_ == "file")
  {
    open3d::io::ReadTriangleMesh(input_mesh_path_, *o3d_mesh);
  }
  else
  {
    std::cerr << "Invalid mesh type argument" << std::endl;
    rclcpp::shutdown();
  }
  open3d::io::WriteTriangleMesh(output_mesh_path_, *o3d_mesh);
  if (mesh_type_ == "file")
  {
    visualizer_->AddPatientMesh(output_mesh_path_, skin_mesh_path_);
  }
  else
  {
    visualizer_->AddMesh(output_mesh_path_, 0);
  }
  // skin mesh path ignored if not file
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Loaded mesh with "
                         << o3d_mesh->vertices_.size() << " vertices and "
                         << o3d_mesh->triangles_.size() << " triangles.");
  if (o3d_mesh->vertices_.empty())
  {
    std::cerr << "The loaded mesh contains no vertices." << std::endl;
    rclcpp::shutdown();
  }
  o3d_mesh->ComputeTriangleNormals();
  o3d_mesh->OrientTriangles();
  o3d_mesh->NormalizeNormals();

  RCLCPP_INFO_STREAM(this->get_logger(), "Computing mesh properties...");

  if(mesh_type_ == "file"){
    for(size_t i = 0; i < o3d_mesh->triangles_.size(); i++){
        auto face = o3d_mesh->triangles_[i];
        int v0 = face[0];
        int v1 = face[1];
        int v2 = face[2];
        auto triangle_center = (o3d_mesh->vertices_[v0] + o3d_mesh->vertices_[v1] + o3d_mesh->vertices_[v2]) / 3;
        auto direction = (triangle_center - o3d_mesh->GetCenter()).normalized();
        if (o3d_mesh->triangle_normals_[i].dot(direction) < 0){
            o3d_mesh->triangle_normals_[i] *= -1;
        }

    }
  }
  mesh_ = std::make_shared<Mesh>(o3d_mesh->vertices_, o3d_mesh->triangles_,
                                 o3d_mesh->triangle_normals_);
  rclcpp::sleep_for(1s); // idk why it is needed

  Eigen::Vector3d delta_x = enforce_virtual_fixture(
      *mesh_, x_new_, vf_pose_, radius_, constraint_planes_, lookup_area_, *visualizer_);
  vf_pose_ += 0.9 * delta_x;

  visualizer_->AddPatientMesh(output_mesh_path_, skin_mesh_path_);
  // Start the keyboard input thread
  std::thread input_thread(&VFControl::KeyboardInputLoop, this);
  input_thread.detach();

  RCLCPP_INFO(this->get_logger(),
              "\033[0;32mVisualization thread started\033[0m");
}

// Function to handle keyboard input
void VFControl::KeyboardInputLoop()
{
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
  while (rclcpp::ok())
  {
    c = getchar();

    // WASD + Arrow keys control
    if (c == 'w')
      direction(1) += 1;
    if (c == 's')
      direction(1) -= 1;
    if (c == 'a')
      direction(0) -= 1;
    if (c == 'd')
      direction(0) += 1;
    if (c == 'q')
      direction(2) += 1;
    if (c == 'e')
      direction(2) -= 1;
    x_new_ = x_new_ + direction * 0.003;
    Eigen::Vector3d delta_x = enforce_virtual_fixture(
        *mesh_, x_new_, vf_pose_, radius_, constraint_planes_, lookup_area_, *visualizer_);
    visualizer_->UpdateScene(constraint_planes_, x_new_, vf_pose_, radius_);
    vf_pose_ += 0.9 * delta_x;
    direction << 0.0, 0.0, 0.0;
  }

  // Restore the old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}
int main(int argc, char **argv)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting VF Control node");

  rclcpp::init(argc, argv);
  auto node = std::make_shared<VFControl>();
  node->Initialize();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling impedance service:");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}