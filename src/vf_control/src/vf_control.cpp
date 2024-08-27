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

    x_new_ << 0.0, 0.0, 0.0;
    vf_pose_ << 0.0, 0.0, 0.0;
    // Initializes the TF2 transform listener and buffer
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    this->mesh_type_ = this->get_parameter("mesh_type").as_string();
    this->input_mesh_path_ = this->get_parameter("input_mesh_path").as_string();
    this->output_mesh_path_ = this->get_parameter("output_mesh_path").as_string();
    this->skin_mesh_path_ = this->get_parameter("skin_mesh_path").as_string();
    this->radius_ = this->get_parameter("radius").as_double();
    this->lookup_area_ = this->get_parameter("lookup_area").as_double();
}
void VFControl::Initialize()
{
    auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    visualizer_ = std::make_shared<Visualizer>(this->shared_from_this(), base_link_name_);

    if (mesh_type_ == "bunny")
    {
        open3d::data::BunnyMesh dataset;
        o3d_mesh = open3d::io::CreateMeshFromFile(dataset.GetPath());
    }
    else if (mesh_type_ == "knot")
    {
        open3d::data::KnotMesh dataset;
        o3d_mesh = open3d::io::CreateMeshFromFile(dataset.GetPath());
        o3d_mesh->Scale(0.002, Eigen::Vector3d(0, 0, 0));
    }
    else if (mesh_type_ == "sphere")
    {
        x_new_ << -0.09, -0.0024, 0.20140033;
        vf_pose_ << -0.09, -0.0024, 0.20140033;
        o3d_mesh = open3d::geometry::TriangleMesh::CreateSphere(0.2, 20);
    }
    else if (mesh_type_ == "file")
    {
        // auto o3d_mesh_tmp = std::make_shared<open3d::geometry::TriangleMesh>();
        // open3d::io::ReadTriangleMesh(input_mesh_path_, *o3d_mesh_tmp);
        // auto mesh_tmp = std::make_shared<Mesh>(o3d_mesh_tmp->vertices_, o3d_mesh_tmp->triangles_, o3d_mesh_tmp->triangle_normals_);
        // mesh_tmp->extrudeMeshRadially(Eigen::Vector3d(0, 0, 0), 0.05);
        // o3d_mesh->vertices_ = mesh_tmp->vertices;
        // o3d_mesh->triangles_ = mesh_tmp->faces;
        open3d::io::ReadTriangleMesh(input_mesh_path_, *o3d_mesh);
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
    }else{
        visualizer_->AddMesh(output_mesh_path_, 0);
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

    RCLCPP_INFO_STREAM(this->get_logger(), "Computing mesh properties...");
    mesh_ = std::make_shared<Mesh>(o3d_mesh->vertices_, o3d_mesh->triangles_, o3d_mesh->triangle_normals_);
}
void VFControl::out_virtuose_pose_CB(
    const raptor_api_interfaces::msg::OutVirtuosePose::SharedPtr msg)
{
    x_new_ << msg->virtuose_pose.translation.x, msg->virtuose_pose.translation.y,
        msg->virtuose_pose.translation.z;

    Eigen::Vector3d delta_x = enforce_virtual_fixture(*mesh_, x_new_, vf_pose_, radius_, constraint_planes_, lookup_area_);
    vf_pose_ += 0.9 * delta_x;
    visualizer_->UpdateScene(constraint_planes_, x_new_, vf_pose_, radius_);
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
}
void VFControl::out_virtuose_statusCB(
    const raptor_api_interfaces::msg::OutVirtuoseStatus::SharedPtr msg)
{
    (void)msg;
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

int main(int argc, char **argv)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting VF Control node");

    rclcpp::init(argc, argv);
    auto node = std::make_shared<VFControl>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling impedance service:");
    node->Initialize();
    node->call_impedance_service();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}