geometry_msgs::msg::Point vector3_to_point(
    const geometry_msgs::msg::Vector3 &vec) {
  geometry_msgs::msg::Point point;
  point.x = vec.x;
  point.y = vec.y;
  point.z = vec.z;
  return point;
}
// inline double filter_force(double alpha, double v, double v_old) {
//   return alpha * v_old + (1 - alpha) * v;
// }