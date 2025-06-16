#ifndef VF_ENFORCER_HPP
#define VF_ENFORCER_HPP
#include <omp.h>
#include <Eigen/Core>
#include <geometry_msgs/msg/wrench.hpp>
#include <rclcpp/rclcpp.hpp>
#include "mesh_virtual_fixtures/mesh.hpp"
#include "mesh_virtual_fixtures/vf_computation.hpp"
#include "utils/conversions.hpp"
#include "utils/geometry.hpp"
#include "utils/json.hpp"
#include "utils/visualization.hpp"

class VFEnforcer {
 public:
  VFEnforcer(std::shared_ptr<rclcpp::Node> node, Eigen::Vector3d x_des,
             std::string base_link_name, double tool_vis_radius,
             std::shared_ptr<Visualizer> visualizer) {
    node_ = node;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Starting VF Control node with name vf_control");
    mesh_type_ = node_->get_parameter("vf_parameters.mesh_type").as_string();
    input_mesh_path_ =
        node_->get_parameter("vf_parameters.input_mesh_path").as_string();
    output_mesh_path_ =
        node_->get_parameter("vf_parameters.output_mesh_path").as_string();
    skin_mesh_path_ =
        node_->get_parameter("vf_parameters.skin_mesh_path").as_string();
    skin_params_path_ =
        node_->get_parameter("vf_parameters.skin_params_path").as_string();
    RCLCPP_INFO_STREAM(node->get_logger(), "\n\n \033Skin mesh path: "
                                               << skin_mesh_path_ << "\033[0m");
    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "\033Input mesh path: " << input_mesh_path_ << "\033[0m \n\n");
    tool_radius_ =
        node_->get_parameter("vf_parameters.tool_radius").as_double();
    tool_vis_radius_ = tool_vis_radius;
    lookup_area_ =
        node_->get_parameter("vf_parameters.lookup_area").as_double();
    plane_size_ = node_->get_parameter("vf_parameters.plane_size").as_double();

    vf_pos_vibration_amplitude_ =
        node_->get_parameter("vf_parameters.pos_vibration_amplitude")
            .as_double();

    auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    visualizer_ = visualizer;

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
      skin_mesh_ = std::make_shared<open3d::geometry::TriangleMesh>();
      open3d::io::ReadTriangleMesh(skin_mesh_path_, *skin_mesh_);
      skin_mesh_->OrientTriangles();
      skin_mesh_->ComputeTriangleNormals();
      std::vector<Eigen::Vector3d> centroids = getCentroids(*skin_mesh_);
      // create point cloud from centroids
      skin_pcd.points_ = centroids;
      skin_mesh_transform_ = utils::read_transform_from_file(skin_params_path_);
      RCLCPP_INFO_STREAM(node_->get_logger(), "Skin mesh transform matrix: \n"
                                                  << skin_mesh_transform_
                                                  << "\n");
      skin_kdtree_.SetGeometry(skin_pcd);
      ribs_lateral_extension_ = skin_mesh_transform_.block<3, 1>(0, 2);
      visualizer_->add_patient_mesh(output_mesh_path_, skin_mesh_path_);
    } else {
      visualizer_->add_mesh(output_mesh_path_, 0);
    }
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Loaded mesh with "
                           << o3d_mesh->vertices_.size() << " vertices and "
                           << o3d_mesh->triangles_.size() << " triangles.");
    // Ensure the mesh has vertices
    if (o3d_mesh->vertices_.empty()) {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
                          "Mesh has no vertices, shutting down");
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

    RCLCPP_INFO_STREAM(node_->get_logger(), "Computing mesh properties...");
    mesh_ = std::make_shared<Mesh>(o3d_mesh->vertices_, o3d_mesh->triangles_,
                                   o3d_mesh->triangle_normals_);
    visualizer_->update_scene(constraint_planes_, x_des, x_old_,
                              tool_vis_radius_, plane_size_);
  }

  std::pair<Eigen::Vector3d, geometry_msgs::msg::Wrench> enforce_vf(
      const Eigen::Vector3d& x_des) {
    auto [delta_x, in_contact] = compute_vf::enforce_virtual_fixture(
        *mesh_, x_des, x_old_, tool_radius_, constraint_planes_, lookup_area_);
    delta_x_ = delta_x;
    // slow down reference when close to the subject body
    bool slow_down_when_close_to_body_ = true;
    if (slow_down_when_close_to_body_) {
      std::vector<int> indices;
      std::vector<double> distances;
      const double lookup_area_ = 0.05;
      skin_kdtree_.SearchRadius(x_old_, lookup_area_, indices, distances);
      if ((int)indices.size() > 0) {
        // Scale down the delta_x vector to reduce the speed
        // when the probe is close to the body
        double step_size = std::min(delta_x_.norm(), 0.00003);
        delta_x_ = step_size * delta_x_.normalized();
        // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        //                      "Slow down reference when close to body: %f",
        //                      delta_x_.norm());
      }
    }
    // integrate the dynamical system
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
                              tool_vis_radius_, plane_size_);
    geometry_msgs::msg::Wrench overlay_wrench;
    double vibration_force_z = 0.0;
    if (in_contact) {
      vibration_force_z =
          vf_pos_vibration_amplitude_ > 1e-6
              ? vf_pos_vibration_amplitude_ *
                    std::sin(2 * M_PI * 80 *
                             node_->get_clock()->now().seconds())
              : 0.0;
    }
    overlay_wrench.force.x = 0.0;
    overlay_wrench.force.y = 0.0;
    overlay_wrench.force.z = vibration_force_z;
    return std::make_pair(x_new, overlay_wrench);
  }

  std::vector<Eigen::Vector3d> getCentroids(
      const open3d::geometry::TriangleMesh& mesh) {
    std::vector<Eigen::Vector3d> centroids;
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& triangle : mesh.triangles_) {
      centroid = (mesh.vertices_[triangle[0]] + mesh.vertices_[triangle[1]] +
                  mesh.vertices_[triangle[2]]) /
                 3.0;
      centroids.push_back(centroid);
    }
    return centroids;
  }

  bool computeQRef(Eigen::Quaterniond& q_ref, int num_faces) {
    // Compute the normals of the nearest x triangles to x_new with KdTree
    std::vector<int> indices;
    std::vector<double> distances;
    const double lookup_area_ = 0.3;
    skin_kdtree_.SearchRadius(x_old_, lookup_area_, indices, distances);
    if ((int)indices.size() < num_faces) {
      // not enough faces found
      return false;
    }
    Eigen::Vector3d n_avg = Eigen::Vector3d::Zero();

    int faces_idx;
    Eigen::Vector3d normal, z_axis;
    z_axis << 0.0, 0.0, 1.0;  // default x axis
    for (faces_idx = 0; faces_idx < (int)indices.size(); faces_idx++) {
      if (faces_idx > num_faces) {
        break;
      }
      normal = skin_mesh_->triangle_normals_[indices[faces_idx]];
      if (z_axis.dot(normal) < 0.0) {
        // if the normal is pointing in the opposite direction of the x axis,
        // flip it
        RCLCPP_INFO_STREAM(
            rclcpp::get_logger("rclcpp"),
            "Flipping normal " << faces_idx << ": " << normal.transpose());
        normal *= -1.0;
      }
      n_avg += normal;
    }
    // average normals
    n_avg /= faces_idx;
    // normalize and invert sign since face normals point outwards
    n_avg = -n_avg.normalized();
    Eigen::Matrix3d rotmat = rotationMatrixFromVectorZ(n_avg);
    z_axis = rotmat.col(2);
    // pick the x axis of the rotmat as the projection of the ribs axis on the
    // plane defined by z axis
    Eigen::Vector3d x_axis = projectOntoPlane(ribs_lateral_extension_, z_axis);
    x_axis.normalize();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();
    rotmat.col(0) = x_axis;
    rotmat.col(1) = y_axis;
    rotmat.col(2) = z_axis;

    q_ref = Eigen::Quaterniond(rotmat);
    return true;
  }
  Eigen::Vector3d projectOntoPlane(const Eigen::Vector3d& v,
                                   const Eigen::Vector3d& plane_normal) {
    Eigen::Vector3d n = plane_normal.normalized();
    return v - (v.dot(n)) * n;
  }
  Eigen::Quaterniond enforce_orientation_constraints(
      Eigen::Quaterniond& target_orientation, Eigen::Quaterniond& q_old,
      Eigen::Vector3d& thetas) {
    if (first_time_) {
      q_ref_old_ = target_orientation;  // q_ref;
      first_time_ = false;
    }
    Eigen::Quaterniond q_ref;
    bool succes = computeQRef(q_ref, 30);
    if (!succes) {
      // not enough faces found, do not filter
      return target_orientation;
    }
    // Slerp towards the new q_ref
    q_ref = q_ref_old_.slerp(0.1, q_ref);
    bool draw_cones = true;
    if (draw_cones) {
      auto axis = utils::get_quaternions_from_rotmat_axis(q_ref);
      visualizer_->draw_cone(x_old_, axis[0], thetas(0), 0,
                             utils::colors::COLOR_RED);
      visualizer_->draw_cone(x_old_, axis[1], thetas(1), 1,
                             utils::colors::COLOR_GREEN);
      visualizer_->draw_cone(x_old_, axis[2], thetas(2), 2,
                             utils::colors::COLOR_BLUE);
    }
    q_ref_old_ = q_ref;
    q_opt_ =
        conic_cbf::cbfOrientFilter(q_ref, q_old, target_orientation, thetas);
    return q_opt_;
  }

  // private:
  std::shared_ptr<rclcpp::Node> node_;
  // MESH
  std::shared_ptr<Mesh> mesh_;
  // Visualizer visualizer_;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> constraint_planes_;
  // VARIABLES
  std::string input_mesh_path_, output_mesh_path_, skin_mesh_path_,
      skin_params_path_, mesh_type_;
  std::shared_ptr<open3d::geometry::TriangleMesh> skin_mesh_;
  open3d::geometry::KDTreeFlann skin_kdtree_;
  open3d::geometry::PointCloud skin_pcd;
  std::shared_ptr<Visualizer> visualizer_;
  double plane_size_;
  int client__id_;
  int ctr_;
  double tool_radius_, tool_vis_radius_, lookup_area_;
  double vf_pos_vibration_amplitude_;
  Eigen::Vector3d x_old_, delta_x_;
  Eigen::Quaterniond q_opt_, q_ref_old_;
  Eigen::Matrix4d skin_mesh_transform_;
  Eigen::Vector3d ribs_lateral_extension_;
  bool first_time_ = true;
};

#endif  // VF_ENFORCER_HPP