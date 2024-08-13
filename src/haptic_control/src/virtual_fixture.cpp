#include "virtual_fixture.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>

VirtualFixtureCalculator::VirtualFixtureCalculator(std::string load_path) {

  // Load and prepare the surface
  surface = std::make_shared<open3d::t::geometry::TriangleMesh>();
  open3d::t::io::ReadTriangleMesh(load_path, *surface);

  // Initialize raycasting
  scene = std::make_shared<open3d::t::geometry::RaycastingScene>();
  scene->AddTriangles(*surface);
}

Eigen::Vector3d VirtualFixtureCalculator::EnforceVirtualFixture(
    const Eigen::Vector3d &sphere_center,
    const double sphere_radius) {
  Eigen::Vector3d closest_point;
  double distance = PointToMeshDistance(sphere_center, closest_point);
  std::cout<<"Distance to mesh: "<<distance<<std::endl;
  if (distance < sphere_radius) {
    Eigen::Vector3d direction = (sphere_center - closest_point).normalized();
    return closest_point + direction * sphere_radius;
  }
  return sphere_center;
}

double VirtualFixtureCalculator::PointToMeshDistance(
    const Eigen::Vector3d &point,
    Eigen::Vector3d &closest_point) {
  open3d::core::Tensor query_point = open3d::core::Tensor::Zeros({1,3}, open3d::core::Dtype::Float32);
  query_point[0][0] = point[0];
  query_point[0][1] = point[1];
  query_point[0][2] = point[2];
  std::cout << "Query point: " << query_point.ToString() << std::endl;

  std::unordered_map<std::string, open3d::core::Tensor> ans =
      scene->ComputeClosestPoints(query_point);
  auto closest_points = ans["points"];
  auto eigen_closest_points = open3d::core::eigen_converter::TensorToEigenVector3dVector(closest_points);
  closest_point =
      Eigen::Vector3d(eigen_closest_points[0][0], eigen_closest_points[0][1],
                      eigen_closest_points[0][2]);
  return (point - closest_point).norm();
}

