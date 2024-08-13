#include "virtual_fixture.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>
VirtualFixtureCalculator::VirtualFixtureCalculator(std::string load_path) {
  // Initialization
  std::cout << "Starting VF initialization" << std::endl;

  // Open3D visualization setup
  // visualizer = std::make_shared<open3d::visualization::Visualizer>();
  // visualizer->CreateVisualizerWindow("Virtual Fixture Demo");
  // auto render_option = visualizer->GetRenderOption();
  // render_option.show_coordinate_frame_ = true;
  // render_option.mesh_show_wireframe_ = true;

  // Load and prepare the surface
  surface = std::make_shared<open3d::geometry::TriangleMesh>();
  open3d::io::ReadTriangleMesh(load_path, *surface);
  // visualizer->AddGeometry(surface);
  Eigen::Vector3d translation(0, 0.5, 0);
  surface->Translate(translation, false);
  // visualizer->UpdateGeometry();

  // Initialize raycasting
  scene = std::make_shared<open3d::t::geometry::RaycastingScene>();
  auto mesh_new = open3d::t::geometry::TriangleMesh::FromLegacy(*surface);
  scene->AddTriangles(mesh_new);

  // Initialize sphere properties
  sphere_center = surface->GetCenter() + Eigen::Vector3d(0.05, 0.1, 0.16);
  sphere_center = EnforceVirtualFixture(sphere_center, POINT_RADIUS);

  // sphere = open3d::geometry::TriangleMesh::CreateSphere(POINT_RADIUS);
  // sphere->Translate(sphere_center);
  // sphere->PaintUniformColor({0.1, 0.1, 0.7});
  // visualizer->AddGeometry(sphere);

  // reference_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.1);
  // visualizer->AddGeometry(reference_frame);


}

Eigen::Vector3d VirtualFixtureCalculator::EnforceVirtualFixture(
    const Eigen::Vector3d &sphere_center,
    const double sphere_radius) {
  Eigen::Vector3d closest_point;
  double distance = PointToMeshDistance(sphere_center, closest_point);
  

  if (distance < sphere_radius) {
    Eigen::Vector3d direction = (sphere_center - closest_point).normalized();
    return closest_point + direction * sphere_radius;
  }

//   if (!IsSphereOutside(sphere_center, surface, sphere_radius)) {
//     Eigen::Vector3d direction = (sphere_center - closest_point).normalized();
//     return closest_point + direction * sphere_radius;
//   }

  return sphere_center;
}

// bool VirtualFixtureCalculator::IsSphereOutside(
//     const Eigen::Vector3d &sphere_center,
//     const std::shared_ptr<open3d::geometry::TriangleMesh> &surface,
//     double sphere_radius) {
//   Eigen::Vector3d closest_point;
//   double distance = PointToMeshDistance(sphere_center, closest_point);
//   return distance >= sphere_radius;
// }

double VirtualFixtureCalculator::PointToMeshDistance(
    const Eigen::Vector3d &point,
    Eigen::Vector3d &closest_point) {
  open3d::core::Tensor query_point =

// open3d::Tensor::Tensor	(	const std::vector< T > & 	init_vals,
// const SizeVector & 	shape,
// Dtype 	dtype,
// const Device & 	device = Device("CPU:0") 
// )	
  // open3d::core::Tensor({point[0], point [1], point[2]}, {3}, open3d::core::Dtype::Float32);

  open3d::core::Tensor::Zeros({1,3}, open3d::core::Dtype::Float32);
  query_point[0][0] = point[0];
  query_point[0][1] = point[1];
  query_point[0][2] = point[2];
  std::cout << "Query point: " << query_point.ToString() << std::endl;

  std::unordered_map<std::string, open3d::core::Tensor> ans =
      scene->ComputeClosestPoints(query_point);
  // access field points
  auto closest_points = ans["points"][0];
  auto eigen_closest_points =
      open3d::core::eigen_converter::TensorToEigenVector3dVector(
          closest_points);
  closest_point =
      Eigen::Vector3d(eigen_closest_points[0][0], eigen_closest_points[0][1],
                      eigen_closest_points[0][2]);
  return (point - closest_point).norm();
}

void VirtualFixtureCalculator::MoveSphere(const Eigen::Vector3d &direction_vector) {
  sphere_center += direction_vector * MOVEMENT_SPEED;
  sphere_center = EnforceVirtualFixture(sphere_center, POINT_RADIUS);
  // sphere->Translate(sphere_center);
  // visualizer->UpdateGeometry();
}

void VirtualFixtureCalculator::RunMainLoop() {
  bool running = true;
  while (running) {
    // Handle events, update sphere, etc.
    // For simplicity, we'll just sleep in this example.
    // visualizer->PollEvents();
    // visualizer->UpdateRender();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
