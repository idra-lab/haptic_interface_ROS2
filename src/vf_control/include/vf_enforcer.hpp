#ifndef VF_ENFORCER_HPP
#define VF_ENFORCER_HPP
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "mesh.hpp"
#include "visualization.hpp"
#include "vf_computation.hpp"
#include <omp.h>

class VFEnforcer
{
public:
    VFEnforcer(std::shared_ptr<rclcpp::Node> node, Eigen::Vector3d x_des)
    {
        node_ = node;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting VF Control node with name vf_control");
        this->mesh_type_ = node_->get_parameter("mesh_type").as_string();
        this->input_mesh_path_ = node_->get_parameter("input_mesh_path").as_string();
        this->output_mesh_path_ = node_->get_parameter("output_mesh_path").as_string();
        this->skin_mesh_path_ = node_->get_parameter("skin_mesh_path").as_string();
        this->radius_ = node_->get_parameter("radius").as_double();
        this->lookup_area_ = node_->get_parameter("lookup_area").as_double();
        this->plane_size_ = node_->get_parameter("plane_size").as_double();

        auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        visualizer_ = std::make_shared<Visualizer>(node_, "base_link", plane_size_);

        if (mesh_type_ == "bunny")
        {
            open3d::data::BunnyMesh dataset;
            o3d_mesh = open3d::io::CreateMeshFromFile(dataset.GetPath());
            o3d_mesh->Translate(Eigen::Vector3d(0, 0.2, 0));
        }
        else if (mesh_type_ == "knot")
        {
            open3d::data::KnotMesh dataset;
            o3d_mesh = open3d::io::CreateMeshFromFile(dataset.GetPath());
            o3d_mesh->Scale(0.002, Eigen::Vector3d(0, 0, 0));
        }
        else if (mesh_type_ == "sphere")
        {
            o3d_mesh = open3d::geometry::TriangleMesh::CreateSphere(0.02, 20);
            o3d_mesh->Translate(Eigen::Vector3d(0, 0.25, 0));

        }
        else if (mesh_type_ == "file")
        {
            open3d::io::ReadTriangleMesh(input_mesh_path_, *o3d_mesh);
            // open3d::visualization::DrawGeometries({o3d_mesh});
            // RCLCPP_INFO_STREAM(node->get_logger(), "Extruded mesh with " << o3d_mesh->vertices_.size() << " vertices and " << o3d_mesh->triangles_.size() << " triangles.");
        }
        else
        {
            std::cerr << "Invalid mesh type argument" << std::endl;
            rclcpp::shutdown();
        }
        open3d::io::WriteTriangleMesh(output_mesh_path_, *o3d_mesh);
        if (mesh_type_ == "file")
        {
            visualizer_->AddPatientMesh(output_mesh_path_, skin_mesh_path_);
        }
        else
        {
            visualizer_->AddMesh(output_mesh_path_, 0);
        }
        RCLCPP_INFO_STREAM(node->get_logger(), "Loaded mesh with " << o3d_mesh->vertices_.size()
                                                                   << " vertices and " << o3d_mesh->triangles_.size() << " triangles.");
        // Ensure the mesh has vertices
        if (o3d_mesh->vertices_.empty())
        {
            std::cerr << "The loaded mesh contains no vertices." << std::endl;
            rclcpp::shutdown();
        }
        o3d_mesh->RemoveDuplicatedVertices();
        o3d_mesh->ComputeTriangleNormals();
        o3d_mesh->OrientTriangles();
        o3d_mesh->NormalizeNormals();

        if (mesh_type_ == "file")
        {
            // make normals point outwards
            // for (size_t i = 0; i < o3d_mesh->triangles_.size(); i++)
            // {
            //     o3d_mesh->triangle_normals_[i] *= -1;
            // }
        }
        RCLCPP_INFO_STREAM(node->get_logger(), "Computing mesh properties...");
        mesh_ = std::make_shared<Mesh>(o3d_mesh->vertices_, o3d_mesh->triangles_, o3d_mesh->triangle_normals_);
        this->x_old_ = x_des;
        this->x_des_old = x_des;
        this->delta_x_ << 0, 0, 0;

        visualizer_->UpdateScene(constraint_planes_, x_des, x_old_, radius_);
    }

    Eigen::Vector3d EnforceVF(Eigen::Vector3d x_des)
    {
        if((x_des - x_des_old).norm() < 0.0005)
        {
            // return last delta_x_
            return Eigen::Vector3d(0,0,0);
        }
        delta_x_ = compute_vf::enforce_virtual_fixture(*mesh_, x_des, x_old_, radius_, constraint_planes_, lookup_area_, *visualizer_);
        auto x_new = x_old_ + delta_x_;
        x_old_ = x_new;
        x_des_old = x_des;
        geometry_msgs::msg::PoseStamped target_pose_vf;
        target_pose_vf.header.stamp = node_->now();
        target_pose_vf.header.frame_id = "base_link";
        target_pose_vf.pose.position.x = x_new[0];
        target_pose_vf.pose.position.y = x_new[1];
        target_pose_vf.pose.position.z = x_new[2];
        target_pose_vf.pose.orientation.w = 1.0;
        visualizer_->UpdateScene(constraint_planes_,x_des,x_new,radius_);
        return delta_x_;
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
    double radius_, lookup_area_;
    Eigen::Vector3d x_old_, delta_x_, x_des_old;
};

#endif // VF_ENFORCER_HPP