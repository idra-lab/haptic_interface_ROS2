#ifndef UTILS_GEOMETRY_HPP
#define UTILS_GEOMETRY_HPP
#include <Eigen/Geometry>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include "colors.hpp"
namespace utils {

double f_linear(double x, double r) {
  double diff = std::abs(x - r);
  if (diff < 0.05) {
    return 1.0 - diff / 0.05;
  } else {
    return 0.0;
  }
};
void project_target_on_sphere(Eigen::Vector3d& target_position_vec,
                              const Eigen::Vector3d& center,
                              const double safety_sphere_radius_) {
  // target_position_vec =
  //     target_position_vec.normalized() * safety_sphere_radius_;
  Eigen::Vector3d sphere_center_to_point = target_position_vec - center;
  sphere_center_to_point =
      sphere_center_to_point.normalized() * safety_sphere_radius_;
  target_position_vec = center + sphere_center_to_point;
}
// extract quaternion from direction
Eigen::Quaterniond quaternion_from_vector3(const Eigen::Vector3d& direction) {
  Eigen::Quaterniond q;
  q.setFromTwoVectors(Eigen::Vector3d(0, 0, 1), direction);
  return q;
}
// read a 4x4 matrix from file
Eigen::Matrix4d read_transform_from_file(std::string path) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open()) {
    std::cerr << "Error opening params file: " << path << std::endl;
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
std::vector<Eigen::Quaterniond> get_quaternions_from_rotmat_axis(
    const Eigen::Quaterniond& q_ref_new) {
  // visualize result
  Eigen::Matrix3d rotmat = q_ref_new.toRotationMatrix();
  Eigen::Vector3d x_axis = rotmat.col(0);
  Eigen::Vector3d y_axis = rotmat.col(1);
  Eigen::Vector3d z_axis = rotmat.col(2);
  auto ori_x = utils::quaternion_from_vector3(x_axis);
  auto ori_y = utils::quaternion_from_vector3(y_axis);
  auto ori_z = utils::quaternion_from_vector3(z_axis);
  return {ori_x, ori_y, ori_z};
}
void sum_wrenches(geometry_msgs::msg::Wrench& w1,
                const geometry_msgs::msg::Wrench& w2) {
  w1.force.x += w2.force.x;
  w1.force.y += w2.force.y;
  w1.force.z += w2.force.z;
  w1.torque.x += w2.torque.x;
  w1.torque.y += w2.torque.y;
  w1.torque.z += w2.torque.z;
}
} // namespace utils
#endif  // UTILS_GEOMETRY_HPP