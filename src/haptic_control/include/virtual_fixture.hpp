#ifndef VIRTUAL_FIXTURE_CALCULATOR_HPP
#define VIRTUAL_FIXTURE_CALCULATOR_HPP

#include "open3d/Open3D.h"
#include "open3d/t/geometry/RaycastingScene.h"
#include "open3d/t/io/TriangleMeshIO.h"
#include <Eigen/Dense>

#include <memory>

class VirtualFixtureCalculator {
public:
  VirtualFixtureCalculator(std::string load_path);
  Eigen::Vector3d EnforceVirtualFixture(const Eigen::Vector3d &sphere_center,
                                        const double sphere_radius);
  double PointToMeshDistance(const Eigen::Vector3d &point,
                             Eigen::Vector3d &closest_point);

private:
  std::shared_ptr<open3d::t::geometry::TriangleMesh> surface;
  std::shared_ptr<open3d::t::geometry::RaycastingScene> scene;
  std::shared_ptr<open3d::geometry::TriangleMesh> sphere;
  Eigen::Vector3d sphere_center;
  const double POINT_RADIUS = 0.01;
  const double MOVEMENT_SPEED = 0.003;
};

#endif // VIRTUAL_FIXTURE_CALCULATOR_HPP