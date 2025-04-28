#ifndef UTILS_GEOMETRY_HPP
#define UTILS_GEOMETRY_HPP
#include <Eigen/Geometry>
#include <cmath>
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
}  // namespace utils
#endif  // UTILS_GEOMETRY_HPP