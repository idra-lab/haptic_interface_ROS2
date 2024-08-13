#ifndef VIRTUAL_FIXTURE_TEST_HPP
#define VIRTUAL_FIXTURE_TEST_HPP

#include "open3d/Open3D.h"
#include "open3d/t/geometry/RaycastingScene.h"
#include <Eigen/Dense>

#include <memory>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <limits>
#include <memory>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "virtual_fixture.hpp"

class VirtualFixtureTest : public rclcpp::Node {
public:
  VirtualFixtureTest(
      const std::string &name = "virtual_fixture_control",
      const std::string &namespace_ = "",
      const rclcpp::NodeOptions &options =
          rclcpp::NodeOptions()
              .allow_undeclared_parameters(true)
              .automatically_declare_parameters_from_overrides(true));
    void PoseCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void UpdateRviz(geometry_msgs::msg::PoseStamped pose, double radius, std::vector<double> color, uint id);
    void ClearRviz();
    void UpdateRibCageRviz();

    private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::shared_ptr<VirtualFixtureCalculator> vf_calculator_;
    std::string load_path_;
    double sphere_radius_;
};

#endif // VIRTUAL_FIXTURE_TEST_HPP