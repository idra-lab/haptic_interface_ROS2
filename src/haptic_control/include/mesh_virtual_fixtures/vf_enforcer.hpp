#ifndef VF_ENFORCER_HPP
#define VF_ENFORCER_HPP
#include <omp.h>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include "mesh.hpp"
#include "vf_computation.hpp"
#include "visualization.hpp"

class VFEnforcer {
 public:
  VFEnforcer(std::shared_ptr<rclcpp::Node> node, Eigen::Vector3d x_des,
             std::string base_link_name) {
    node_ = node;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Starting VF Control node with name vf_control");
    mesh_type_ = node_->get_parameter("vf_parameters.mesh_type").as_string();
    input_mesh_path_ = node_->get_parameter("vf_parameters.input_mesh_path").as_string();
    output_mesh_path_ = node_->get_parameter("vf_parameters.output_mesh_path").as_string();
    skin_mesh_path_ = node_->get_parameter("vf_parameters.skin_mesh_path").as_string();
    tool_radius_ = node_->get_parameter("vf_parameters.tool_radius").as_double();
    tool_vis_radius_ =
        node_->get_parameter("vf_parameters.tool_visualization_radius").as_double();
    lookup_area_ = node_->get_parameter("vf_parameters.lookup_area").as_double();
    plane_size_ = node_->get_parameter("vf_parameters.plane_size").as_double();

    auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    visualizer_ =
        std::make_shared<Visualizer>(node_, base_link_name, plane_size_);

    if (mesh_type_ == "bunny") {
      open3d::data::BunnyMesh dataset;
      o3d_mesh = open3d::io::CreateMeshFromFile(dataset.GetPath());
      o3d_mesh->Scale(3.0, Eigen::Vector3d(0, 0, 0));
      o3d_mesh->Translate(Eigen::Vector3d(0.3, -0.2, 0.4));
    } else if (mesh_type_ == "knot") {
      open3d::data::KnotMesh dataset;
      o3d_mesh = open3d::io::CreateMeshFromFile(dataset.GetPath());
      o3d_mesh->Scale(0.002, Eigen::Vector3d(0, 0, 0));
      o3d_mesh->Translate(Eigen::Vector3d(0.4, 0.0, 0.5));
    } else if (mesh_type_ == "sphere") {
      o3d_mesh = open3d::geometry::TriangleMesh::CreateSphere(0.2, 50);
      o3d_mesh->Translate(Eigen::Vector3d(0.4, 0.0, 0.3));
    } else if (mesh_type_ == "file") {
      open3d::io::ReadTriangleMesh(input_mesh_path_, *o3d_mesh);
      // open3d::visualization::DrawGeometries({o3d_mesh});
      // RCLCPP_INFO_STREAM(node->get_logger(), "Extruded mesh with " <<
      // o3d_mesh->vertices_.size() << " vertices and " <<
      // o3d_mesh->triangles_.size() << " triangles.");
    } else {
      std::cerr << "Invalid mesh type argument" << std::endl;
      rclcpp::shutdown();
    }
    open3d::io::WriteTriangleMesh(output_mesh_path_, *o3d_mesh);
    if (mesh_type_ == "file") {
      visualizer_->add_patient_mesh(output_mesh_path_, skin_mesh_path_);
    } else {
      visualizer_->add_mesh(output_mesh_path_, 0);
    }
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Loaded mesh with "
                           << o3d_mesh->vertices_.size() << " vertices and "
                           << o3d_mesh->triangles_.size() << " triangles.");
    // Ensure the mesh has vertices
    if (o3d_mesh->vertices_.empty()) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Mesh has no vertices, shutting down");
      rclcpp::shutdown();
    }
    o3d_mesh->RemoveDuplicatedVertices();
    o3d_mesh->ComputeTriangleNormals();
    o3d_mesh->OrientTriangles();
    o3d_mesh->NormalizeNormals();

    if (mesh_type_ == "file") {
      // make normals point outwards
      // for (size_t i = 0; i < o3d_mesh->triangles_.size(); i++)
      // {
      //     o3d_mesh->triangle_normals_[i] *= -1;
      // }
    }
    x_old_ << x_des[0], x_des[1], x_des[2];
    delta_x_ << 0.0, 0.0, 0.0;
    
    RCLCPP_INFO_STREAM(node->get_logger(), "Computing mesh properties...");
    mesh_ = std::make_shared<Mesh>(o3d_mesh->vertices_, o3d_mesh->triangles_,
                                   o3d_mesh->triangle_normals_);
    visualizer_->update_scene(constraint_planes_, x_des, x_old_,
                              tool_vis_radius_);

  }

  Eigen::Vector3d enforce_vf(Eigen::Vector3d x_des) {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "");
    delta_x_ = compute_vf::enforce_virtual_fixture(
        *mesh_, x_des, x_old_, tool_radius_, constraint_planes_, lookup_area_,
        *visualizer_);
    // integrate
    auto x_new = x_old_ + delta_x_;
    x_old_ = x_new;
    geometry_msgs::msg::PoseStamped target_pose_vf;
    target_pose_vf.header.stamp = node_->now();
    target_pose_vf.header.frame_id = "base_link";
    target_pose_vf.pose.position.x = x_new[0];
    target_pose_vf.pose.position.y = x_new[1];
    target_pose_vf.pose.position.z = x_new[2];
    target_pose_vf.pose.orientation.w = 1.0;
    visualizer_->update_scene(constraint_planes_, x_des, x_new,
                              tool_vis_radius_);
    return x_new;
  }

  // private:
  std::shared_ptr<rclcpp::Node> node_;
  // MESH
  std::shared_ptr<Mesh> mesh_;
  // Visualizer visualizer_;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> constraint_planes_;
  // VARIABLES
  std::string input_mesh_path_, output_mesh_path_, skin_mesh_path_, mesh_type_;

  std::shared_ptr<Visualizer> visualizer_;
  double plane_size_;
  int client__id_;
  int ctr_;
  double tool_radius_, tool_vis_radius_, lookup_area_;
  Eigen::Vector3d x_old_, delta_x_;
};

#endif  // VF_ENFORCER_HPP