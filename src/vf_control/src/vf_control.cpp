#include "vf_control.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
VFControl::VFControl(
    const std::string &name, const std::string &namespace_,
    const rclcpp::NodeOptions &options)
    : Node(name, namespace_, options)
{

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing publishers");
    out_virtuose_status_ =
        this->create_subscription<raptor_api_interfaces::msg::OutVirtuoseStatus>(
            "out_virtuose_status", 1,
            std::bind(&VFControl::out_virtuose_statusCB, this, _1));
    _out_virtuose_pose_ =
        this->create_subscription<raptor_api_interfaces::msg::OutVirtuosePose>(
            "out_virtuose_pose", 1,
            std::bind(&VFControl::out_virtuose_pose_CB, this, _1));

    target_pos_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_frame",
                                                                1);

    client_ = this->create_client<raptor_api_interfaces::srv::VirtuoseImpedance>(
        "virtuose_impedance");

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "visualization_marker", 1);

    x_new_ << 0.0, 0.0, 0.0;
    vf_pose_ << 0.0, 0.0, 0.0;
    // Initializes the TF2 transform listener and buffer
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    open3d::data::KnotMesh dataset;
    auto o3d_mesh = open3d::io::CreateMeshFromFile(dataset.GetPath());
    o3d_mesh->Scale(0.002, Eigen::Vector3d(0, 0, 0));
    // dump mesh
    open3d::io::WriteTriangleMesh(load_path_, *o3d_mesh);
    RCLCPP_INFO(this->get_logger(), "Mesh Dumped");
    AddMesh();

    rclcpp::sleep_for(1s); // idk why it is needed
    RCLCPP_INFO_STREAM(this->get_logger(), "Loading mesh from file: " << load_path_);
    if (!open3d::io::ReadTriangleMesh(load_path_, *o3d_mesh))
    {
        std::cerr << "Failed to load mesh from file: " << load_path_ << std::endl;
        rclcpp::shutdown();
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded mesh with " << o3d_mesh->vertices_.size()
                                                               << " vertices and " << o3d_mesh->triangles_.size() << " triangles.");
    // Ensure the mesh has vertices
    if (o3d_mesh->vertices_.empty())
    {
        std::cerr << "The loaded mesh contains no vertices." << std::endl;
        rclcpp::shutdown();
    }
    o3d_mesh->ComputeTriangleNormals();
    o3d_mesh->OrientTriangles();
    o3d_mesh->NormalizeNormals();

    // Convert vertices to a PointCloud
    auto point_cloud = std::make_shared<open3d::geometry::PointCloud>();
    point_cloud->points_ = o3d_mesh->vertices_;

    RCLCPP_INFO_STREAM(this->get_logger(), "Computing mesh properties...");
    mesh_ = std::make_shared<Mesh>(o3d_mesh->vertices_, o3d_mesh->triangles_, o3d_mesh->triangle_normals_);
}
void VFControl::out_virtuose_pose_CB(
    const raptor_api_interfaces::msg::OutVirtuosePose::SharedPtr msg)
{
    x_new_ << msg->virtuose_pose.translation.x, msg->virtuose_pose.translation.y,
        msg->virtuose_pose.translation.z;

    Eigen::Vector3d delta_x = enforce_virtual_fixture(*mesh_, x_new_, vf_pose_, radius_, constraint_planes_);
    vf_pose_ += 0.9 * delta_x;
}

void VFControl::call_impedance_service()
{

    // Request impedance mode
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending impedance request");

    auto imp = std::make_shared<
        raptor_api_interfaces::srv::VirtuoseImpedance::Request>();
    this->get_parameter("channel", imp->channel);
    this->get_parameter("ff_device_ip_address", imp->ff_device_ip_address);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ff_device_ip_address: %s",
                imp->ff_device_ip_address.c_str());
    this->get_parameter("ff_device_param_file", imp->ff_device_param_file);
    this->get_parameter("local_ip_address", imp->local_ip_address);

    imp->base_frame.translation.x = 0.0;
    imp->base_frame.translation.y = 0.0;
    imp->base_frame.translation.z = 0.0;

    imp->base_frame.rotation.x = 0.0;
    imp->base_frame.rotation.y = 0.0;
    imp->base_frame.rotation.z = 0.0;
    imp->base_frame.rotation.w = 1.0;

    while (!client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "service not available, waiting again...");
    }

    auto result = client_->async_send_request(imp);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        // Store client_ ID given by virtuose_node
        client__id_ = result.get()->client_id;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Our client_ ID is: %d",
                    client__id_);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Failed to call service impedance");
        return;
    }

    if (client__id_ == 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Failed to call service impedance, client__id_ is zero!");
        return;
    }

    ctr_ = 0;

    // Perform impedance loop at 1000 Hz
    // impedanceThread_ = this->create_wall_timer(
    //     1ms, std::bind(&VFControl::impedanceThread, this));
    rclcpp::sleep_for(1s); // idk why it is needed

    visualizationThread_ = this->create_wall_timer(
        10ms, std::bind(&VFControl::UpdateScene, this));
    RCLCPP_INFO(this->get_logger(), "\033[0;32mVisualization thread started\033[0m");
}
void VFControl::out_virtuose_statusCB(
    const raptor_api_interfaces::msg::OutVirtuoseStatus::SharedPtr msg)
{
    // Store last status date
    // auto status_date_nanosec_ = msg->header.stamp.nanosec;
    // auto status_date_sec_ = msg->header.stamp.sec;
    // auto status_state_ = msg->state;
    // auto status_button_ = msg->buttons;
}
void VFControl::impedanceThread()
{
    // Publish target position
    // target_pose_.header.stamp = target_pose_tf_.header.stamp = get_clock()->now();
    // target_pose_.header.frame_id = target_pose_tf_.header.frame_id = base_link_name_;
    // target_pose_tf_.child_frame_id = "haptic_interface_target";

    // target_pose_.pose.position.x = target_pose_tf_.transform.translation.x =
    //     x_new_[0];
    // target_pose_.pose.position.y = target_pose_tf_.transform.translation.y =
    //     x_new_[1];
    // target_pose_.pose.position.z = target_pose_tf_.transform.translation.z =
    //     x_new_[2];

    // target_pose_.pose.orientation.x = target_pose_tf_.transform.rotation.x = 0.0;
    // target_pose_.pose.orientation.y = target_pose_tf_.transform.rotation.y = 0.0;
    // target_pose_.pose.orientation.z = target_pose_tf_.transform.rotation.z = 0.0;
    // target_pose_.pose.orientation.w = target_pose_tf_.transform.rotation.w = 1.0;

    // // send trasnform and publish target pose
    // tf_broadcaster_->sendTransform(target_pose_tf_);
    // target_pos_publisher_->publish(target_pose_);
}
void VFControl::AddMesh()
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_link_name_;
    marker.header.stamp = this->now();
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = "rib_cage";
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.mesh_resource = "file://" + load_path_;
    marker.mesh_use_embedded_materials = true;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    RCLCPP_INFO(this->get_logger(), "Rib cage mesh loaded");
    marker_array.markers.push_back(marker);
    for (int i = 0; i < 20; i++)
    {
        marker_pub_->publish(marker_array);
        rclcpp::sleep_for(100ms);
    }
}

void VFControl::UpdateScene()
{
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = base_link_name_;
    target_pose.header.stamp = get_clock()->now();
    target_pose.pose.position.x = x_new_(0);
    target_pose.pose.position.y = x_new_(1);
    target_pose.pose.position.z = x_new_(2);
    target_pos_publisher_->publish(target_pose);

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_link_name_;
    marker.header.stamp = get_clock()->now();
    // marker.ns = "virtual_fixture";
    marker.id = 1;
    marker.ns = "virtual_fixture";
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = vf_pose_(0);
    marker.pose.position.y = vf_pose_(1);
    marker.pose.position.z = vf_pose_(2);
    marker.scale.x = radius_;
    marker.scale.y = radius_;
    marker.scale.z = radius_;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);
    // marker.ns = "target_pose";
    marker.ns = "target_pose";
    marker.id = 2;
    marker.pose.position.x = x_new_(0);
    marker.pose.position.y = x_new_(1);
    marker.pose.position.z = x_new_(2);
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker_array.markers.push_back(marker);
    // marker_pub_->publish(marker_array);
    // Visualize plane constraints as collapsed boxes
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.ns = "constraint_planes";
    delete_marker.type = visualization_msgs::msg::Marker::DELETEALL;
    // delete all previous markers
    marker_array.markers.push_back(delete_marker);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.0001;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
    for (size_t i = 0; i < constraint_planes_.size(); i++)
    {
        auto n = constraint_planes_[i].first;
        n = n.normalized();
        auto p = constraint_planes_[i].second;
        auto rotation_axis = z_axis.cross(n);
        auto rotation_angle = std::acos(z_axis.dot(n));
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(rotation_angle, rotation_axis);
        marker.id = i;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.pose.position.x = p(0);
        marker.pose.position.y = p(1);
        marker.pose.position.z = p(2);
        marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
}

int main(int argc, char **argv)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting VF Control node");

    rclcpp::init(argc, argv);
    auto node = std::make_shared<VFControl>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling impedance service:");

    node->call_impedance_service();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}