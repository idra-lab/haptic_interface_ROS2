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
    vf_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "virtual_fixture", 1, std::bind(&VFControl::virtual_fixture_cb, this, _1));

    client_ = this->create_client<raptor_api_interfaces::srv::VirtuoseImpedance>(
        "virtuose_impedance");
    
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "visualization_marker", 1);

    x_new_ << 0.0, 0.0, 0.0;
    vf_pose_ << 0.0, 0.0, 0.0;
    // Initializes the TF2 transform listener and buffer
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}
void VFControl::out_virtuose_pose_CB(
    const raptor_api_interfaces::msg::OutVirtuosePose::SharedPtr msg)
{
    x_new_ << msg->virtuose_pose.translation.x, msg->virtuose_pose.translation.y,
        msg->virtuose_pose.translation.z;
}
void VFControl::virtual_fixture_cb(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    vf_pose_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
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
    visualizationThread_ = this->create_wall_timer(
        30ms, std::bind(&VFControl::UpdateScene, this));
    RCLCPP_INFO(this->get_logger(), "\033[0;32mVisualization thread started\033[0m");
    AddMesh();
}
void VFControl::out_virtuose_statusCB(
    const raptor_api_interfaces::msg::OutVirtuoseStatus::SharedPtr msg)
{
    // Store last status date
    auto status_date_nanosec_ = msg->header.stamp.nanosec;
    auto status_date_sec_ = msg->header.stamp.sec;
    auto status_state_ = msg->state;
    auto status_button_ = msg->buttons;
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
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_link_name_;
    marker.header.stamp = this->now();
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
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
    marker_pub_->publish(marker);
}
void VFControl::UpdateScene()
{
    geometry_msgs::msg::PoseStamped vf_pose, target_pose;
    vf_pose.header.frame_id = target_pose.header.frame_id = base_link_name_;
    vf_pose.header.stamp = target_pose.header.stamp = get_clock()->now();
    vf_pose.pose.position.x = vf_pose_(0);
    vf_pose.pose.position.y = vf_pose_(1);
    vf_pose.pose.position.z = vf_pose_(2);
    target_pose.pose.position.x = x_new_(0);
    target_pose.pose.position.y = x_new_(1);
    target_pose.pose.position.z = x_new_(2);

    // Publish virtual fixture and target pose
    // vf_sub_->publish(vf_pose);
    target_pos_publisher_->publish(target_pose);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_link_name_;
    marker.header.stamp = get_clock()->now();
    // marker.ns = "virtual_fixture";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = vf_pose_(0);
    marker.pose.position.y = vf_pose_(1);
    marker.pose.position.z = vf_pose_(2);
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_pub_->publish(marker);
    // marker.ns = "target_pose";
    marker.id = 2;
    marker.pose.position.x = x_new_(0);
    marker.pose.position.y = x_new_(1);
    marker.pose.position.z = x_new_(2);
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_pub_->publish(marker);
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