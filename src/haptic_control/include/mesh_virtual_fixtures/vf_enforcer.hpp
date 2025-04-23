#ifndef VF_ENFORCER_HPP
#define VF_ENFORCER_HPP
#include <omp.h>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include "mesh_virtual_fixtures/mesh.hpp"
#include "mesh_virtual_fixtures/vf_computation.hpp"
#include "utils/conversions.hpp"
#include "utils/json.hpp"
#include "utils/visualization.hpp"

class VFEnforcer {
 public:
  Eigen::Matrix4d get_skin_transform() {
    std::ifstream ifs(skin_params_path_, std::ios::binary);
    if (!ifs.is_open()) {
      std::cerr << "Error opening params file: " << skin_params_path_
                << std::endl;
      rclcpp::shutdown();
      return Eigen::Matrix4d::Identity();
    }

    nlohmann::json j;
    ifs >> j;

    // === Get rotation matrix: joints_ori[0][11] ===
    auto rotmat_json = j["joints_ori"][0][11];

    Eigen::Matrix3d R;
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col) R(row, col) = rotmat_json[row][col];

    // === Get position: joints[0][11] ===
    auto pos_json = j["joints"][0][11];
    Eigen::Vector3d T;
    for (int i = 0; i < 3; ++i) T(i) = pos_json[i];

    // === Compose transformation matrix ===
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = R;
    transform.block<3, 1>(0, 3) = T;

    return transform;
  }

  VFEnforcer(std::shared_ptr<rclcpp::Node> node, Eigen::Vector3d x_des,
             std::string base_link_name, double tool_vis_radius) {
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
      skin_mesh_ = std::make_shared<open3d::geometry::TriangleMesh>();
      open3d::io::ReadTriangleMesh(skin_mesh_path_, *skin_mesh_);
      skin_mesh_->OrientTriangles();
      skin_mesh_->ComputeTriangleNormals();
      std::vector<Eigen::Vector3d> centroids = getCentroids(*skin_mesh_);
      // create point cloud from centroids
      skin_pcd.points_ = centroids;
      skin_mesh_transform_ = get_skin_transform();
      RCLCPP_INFO_STREAM(node->get_logger(), "Skin mesh transform matrix: \n"
                                                 << skin_mesh_transform_
                                                 << "\n");
      skin_kdtree_.SetGeometry(skin_pcd);
      // draw geometries
      // auto reference_frame =
      //     open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.2);
      // reference_frame->Transform(skin_mesh_transform_);

      // open3d::visualization::DrawGeometries({skin_mesh_, reference_frame},
      //                                       "Skin mesh");

      // taking the z vector of the rotation of transform matrix
      ribs_lateral_extension_ = -skin_mesh_transform_.block<3, 1>(0, 2);
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
      RCLCPP_ERROR_STREAM(node->get_logger(),
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

  // def triangle_centroids(mesh):
  //   triangles = np.asarray(mesh.triangles)
  //   vertices = np.asarray(mesh.vertices)
  //   centroids = np.mean(vertices[triangles], axis=1)
  //   return centroids

  std::vector<Eigen::Vector3d> getCentroids(
      open3d::geometry::TriangleMesh& mesh) {
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

  Eigen::Quaterniond computeQRef(Eigen::Quaterniond default_qref,
                                 int num_faces) {
    // Compute the normals of the nearest x triangles to x_new with KdTree
    std::vector<int> indices;
    std::vector<double> distances;
    double lookup_area_ = 0.2;
    skin_kdtree_.SearchRadius(x_old_, lookup_area_, indices, distances);
    if ((int)indices.size() < num_faces) {
      return default_qref;
    }
    // take the closest 30 triangles
    // sort the indices by distance

    Eigen::Vector3d n_avg = Eigen::Vector3d::Zero();

    int faces_idx;
    for (faces_idx = 0; faces_idx < (int)indices.size(); faces_idx++) {
      if (faces_idx > num_faces) {
        break;
      }
      n_avg += skin_mesh_->triangle_normals_[indices[faces_idx]];
      // RCLCPP_INFO_STREAM(
      //     node_->get_logger(),
      // "Adding normal: "
      //     << skin_mesh_->triangle_normals_[indices[faces_idx]].transpose()
      //     << " | index: " << indices[faces_idx]);
    }
    // average normals
    n_avg /= faces_idx;
    // normalize and invert sign since face normals point outwards
    n_avg = -n_avg.normalized();
    Eigen::Matrix3d rotmat = rotationMatrixFromVectorZ(n_avg);
    Eigen::Vector3d z_axis = rotmat.col(2);
    // pick the x axis of the rotmat as the projection of the ribs axis on the
    // plane defined by z axis
    Eigen::Vector3d x_axis = projectOntoPlane(ribs_lateral_extension_, z_axis);
    x_axis.normalize();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();
    rotmat.col(0) = x_axis;
    rotmat.col(1) = y_axis;
    rotmat.col(2) = z_axis;

    Eigen::Quaterniond q_ref(rotmat);
    return q_ref;
  }
  Eigen::Vector3d projectOntoPlane(const Eigen::Vector3d& v,
                                   const Eigen::Vector3d& plane_normal) {
    Eigen::Vector3d n = plane_normal.normalized();
    return v - (v.dot(n)) * n;
  }
  Eigen::Quaterniond enforce_orientation_constraints(
      Eigen::Quaterniond& target_orientation, Eigen::Quaterniond& q_old,
      Eigen::Quaterniond& q_ref, Eigen::Vector3d& thetas, double timestep) {
    if (first_time_) {
      q_ref_old_ = q_ref;
      first_time_ = false;
    }
    Eigen::Quaterniond q_ref_new = computeQRef(q_ref, 30);
    // Slerp towards the new q_ref
    q_ref_new = q_ref_old_.slerp(0.1, q_ref_new);

    // visualize result
    Eigen::Matrix3d rotmat = q_ref_new.toRotationMatrix();
    Eigen::Vector3d x_axis = rotmat.col(0);
    Eigen::Vector3d y_axis = rotmat.col(1);
    Eigen::Vector3d z_axis = rotmat.col(2);
    std::vector<Eigen::Vector3d> axis, origins, colors;
    axis.push_back(x_axis);
    axis.push_back(y_axis);
    axis.push_back(z_axis);
    origins.push_back(x_old_);
    origins.push_back(x_old_);
    origins.push_back(x_old_);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "Computed rotmat: \n"
    //                                             << rotmat << "\n");
    colors.push_back({1.0, 0.0, 0.0});
    colors.push_back({0.0, 1.0, 0.0});
    colors.push_back({0.0, 0.0, 1.0});
    visualizer_->draw_arrows(origins, axis, colors, 0);

    q_ref_old_ = q_ref_new;
    q_opt_ = conic_cbf::cbfOrientFilter(q_ref_new, q_old, target_orientation,
                                        thetas);

    // compute RPY angles
    // Eigen::Vector3d rpy_angles_ref =
    //     q_ref_new.toRotationMatrix().eulerAngles(0, 1, 2);
    // Eigen::Vector3d rpy_angles_target =
    //     target_orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    // auto wrapToPi = [](double angle) {
    //   angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    //   if (angle < 0) angle += 2.0 * M_PI;
    //   return angle - M_PI;
    // };
    // for (int i = 0; i < 3; ++i) {
    //   rpy_angles_target(i) = wrapToPi(rpy_angles_target(i));
    //   rpy_angles_ref(i) = wrapToPi(rpy_angles_ref(i));
    // }
    // RCLCPP_INFO(node_->get_logger(), "RPY target: %f %f %f",
    //             rpy_angles_target(0), rpy_angles_target(1),
    //             rpy_angles_target(2));
    // RCLCPP_INFO(node_->get_logger(), "RPY ref: %f %f %f", rpy_angles_ref(0),
    //             rpy_angles_ref(1), rpy_angles_ref(2));

    // rpy_angles_target(0) =
    //     std::clamp(rpy_angles_target(0), rpy_angles_ref(0) - thetas(0),
    //                rpy_angles_ref(0) + thetas(0));
    // rpy_angles_target(1) =
    //     std::clamp(rpy_angles_target(1), rpy_angles_ref(1) - thetas(1),
    //                rpy_angles_ref(1) + thetas(1));
    // rpy_angles_target(2) =
    //     std::clamp(rpy_angles_target(2), rpy_angles_ref(2) - thetas(2),
    //                rpy_angles_ref(2) + thetas(2));
    // // convert to rotmat
    // Eigen::Matrix3d R;
    // R = Eigen::AngleAxisd(rpy_angles_target(2), Eigen::Vector3d::UnitZ()) *
    //     Eigen::AngleAxisd(rpy_angles_target(1), Eigen::Vector3d::UnitY()) *
    //     Eigen::AngleAxisd(rpy_angles_target(0), Eigen::Vector3d::UnitX());
    // convert to quaternion
    // target_orientation = Eigen::Quaterniond(R);
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
  Eigen::Vector3d x_old_, delta_x_;
  Eigen::Quaterniond q_opt_, q_ref_old_;
  Eigen::Matrix4d skin_mesh_transform_;
  Eigen::Vector3d ribs_lateral_extension_;
  bool first_time_ = true;
};

#endif  // VF_ENFORCER_HPP