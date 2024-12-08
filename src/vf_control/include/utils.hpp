geometry_msgs::msg::Point vector3_to_point(
    const geometry_msgs::msg::Vector3& vec) {
  geometry_msgs::msg::Point point;
  point.x = vec.x;
  point.y = vec.y;
  point.z = vec.z;
  return point;
}