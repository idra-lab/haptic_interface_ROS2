#ifndef VF_ENFORCER_HPP
#define VF_ENFORCER_HPP
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "mesh.hpp"
#include "vf_computation.hpp"
#include "visualization.hpp"
#include <omp.h>

class VFEnforcer
{
public:
    VFEnforcer(std::shared_ptr<rclcpp::Node> node, Eigen::Vector3d x_des);

    void KeyboardInputLoop();
    Eigen::Vector3d EnforceVF(Eigen::Vector3d x_des);

private:
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
    double radius_, lookup_area_;
    Eigen::Vector3d old_x_;
};

#endif // VF_ENFORCER_HPP