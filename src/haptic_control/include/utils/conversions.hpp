#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
Eigen::Quaterniond rosQuatToEigen(const geometry_msgs::msg::Quaternion& q_ros) {
    return Eigen::Quaterniond(q_ros.w, q_ros.x, q_ros.y, q_ros.z);
}
geometry_msgs::msg::Quaternion eigenToRosQuat(const Eigen::Quaterniond& q_eigen) {
    geometry_msgs::msg::Quaternion q_ros;
    q_ros.w = q_eigen.w();
    q_ros.x = q_eigen.x();
    q_ros.y = q_eigen.y();
    q_ros.z = q_eigen.z();
    return q_ros;
}
Eigen::Quaterniond eulerToQuaternion(const geometry_msgs::msg::Vector3& euler) {
    tf2::Quaternion tf_q;
    tf_q.setRPY(euler.x, euler.y, euler.z);  // Roll, Pitch, Yaw
    return Eigen::Quaterniond(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());
}
geometry_msgs::msg::Vector3 quaternionToEuler(const Eigen::Quaterniond& q) {
    tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

    geometry_msgs::msg::Vector3 euler;
    euler.x = roll;
    euler.y = pitch;
    euler.z = yaw;
    return euler;
}
Eigen::Vector3d eigenFromRosPoint(const geometry_msgs::msg::Point& point) {
    return Eigen::Vector3d(point.x, point.y, point.z);
}
geometry_msgs::msg::Point vector3_to_point(
    const geometry_msgs::msg::Vector3 &vec) {
  geometry_msgs::msg::Point point;
  point.x = vec.x;
  point.y = vec.y;
  point.z = vec.z;
  return point;
}
geometry_msgs::msg::Vector3  point_to_vector3(
    const geometry_msgs::msg::Point &point) {
  geometry_msgs::msg::Vector3 vec;
  vec.x = point.x;
  vec.y = point.y;
  vec.z = point.z;
  return vec;
}
geometry_msgs::msg::Point eigenToRosPoint(const Eigen::Vector3d &vec) {
  geometry_msgs::msg::Point point;
  point.x = vec.x();
  point.y = vec.y();
  point.z = vec.z();
  return point;
}