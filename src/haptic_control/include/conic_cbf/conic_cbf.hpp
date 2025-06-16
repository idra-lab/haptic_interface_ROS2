#ifndef CONIC_CBF_QP_HPP
#define CONIC_CBF_QP_HPP
#include <unsupported/Eigen/MatrixFunctions>
#include "mesh_virtual_fixtures/qp_wrapper.hpp"
#include "rclcpp/rclcpp.hpp"
namespace conic_cbf {

Eigen::Matrix3d skew(Eigen::Vector3d &omega) {
  Eigen::Matrix3d omega_skew;
  omega_skew << 0, -omega(2), omega(1), omega(2), 0, -omega(0), -omega(1),
      omega(0), 0;
  return omega_skew;
}
double theta_from_matrix(Eigen::Matrix3d &R) {
  double value = std::clamp((R.trace() - 1) / 2, -1.0, 1.0);
  return std::acos(value);
}
// exponential map from vector omega
Eigen::Matrix3d exp_map(Eigen::Vector3d &omega) {
  double theta = omega.norm();
  if (theta == 0) {
    return Eigen::Matrix3d::Identity();
  }
  Eigen::Matrix3d omega_skew = skew(omega);
  return Eigen::Matrix3d::Identity() + std::sin(theta) / theta * omega_skew +
         (1 - std::cos(theta)) / (theta * theta) * omega_skew * omega_skew;
}
// logaritmic map from rotation matrix
Eigen::Vector3d log_map(Eigen::Matrix3d &R) {
  double theta = theta_from_matrix(R);
  if (theta < 1e-6) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Matrix3d log_R = theta / (2 * std::sin(theta)) * (R - R.transpose());
  return Eigen::Vector3d(log_R(2, 1), log_R(0, 2), log_R(1, 0));
}
// # Define the CBF
inline double h(Eigen::Matrix3d &R, Eigen::Vector3d &e_w,
                Eigen::Vector3d &e_ref, double theta_i) {
  return e_w.transpose() * R.transpose() * e_ref - std::cos(theta_i);
}

// # Define the CBF constraint
// lie derivative of h with respect to g
inline Eigen::Vector3d L_gh(Eigen::Matrix3d &R, Eigen::Vector3d &e_w,
                            Eigen::Vector3d &e_ref) {
  return -e_ref.transpose() * R * skew(e_w);
}

double h_value(const Eigen::Quaterniond &q, const Eigen::Quaterniond &q_ref,
               const Eigen::Vector3d &thetas) {
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Matrix3d R_ref = q_ref.toRotationMatrix();
  Eigen::Vector3d e_w = Eigen::Vector3d::Unit(2);
  Eigen::Vector3d e_ref = R_ref.col(2);
  return h(R, e_w, e_ref, thetas(2));
}

Eigen::Quaterniond cbfOrientFilter(const Eigen::Quaterniond &q_ref,
                                   Eigen::Quaterniond &q,
                                   const Eigen::Quaterniond &q_new,
                                   Eigen::Vector3d &thetas) {
  Eigen::Matrix3d R_ref = q_ref.toRotationMatrix();
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Matrix3d R_new = q_new.toRotationMatrix();

  const int n_constraints = 3;
  const double gamma = 0.01;

  qpOASES::Options qpOptions;
  // suppress output
  qpOptions.printLevel = qpOASES::PL_NONE;
  qpOASES::QProblem min_problem(3, n_constraints,
                                qpOASES::HessianType::HST_IDENTITY);
  min_problem.setOptions(qpOptions);
  // std::cout << "R\n: " << R << std::endl;
  // std::cout << "R new\n: " << R_new << std::endl;
  Eigen::Matrix3d delta_R = R.transpose() * R_new;
  Eigen::Vector3d omega = log_map(delta_R);
  Eigen::Vector3d u_nominal = omega;

  const Eigen::Vector<real_t, 3> g = -u_nominal;

  Eigen::Matrix<real_t, Eigen::Dynamic, 3, Eigen::RowMajor> A(n_constraints, 3);
  Eigen::Vector<real_t, Eigen::Dynamic> A_lb(n_constraints);

  for (int i = 0; i < n_constraints; ++i) {
    Eigen::Vector3d e_ref = R_ref.col(i);
    Eigen::Vector3d e_w = Eigen::Vector3d::Unit(i);
    A.row(i) = L_gh(R, e_w, e_ref).transpose();
    A_lb(i) = -gamma * h(R, e_w, e_ref, thetas(i));
  }

  int nWSR = 200;
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Solving QP");

  auto status =
      min_problem.init(0, g.data(), A.data(), 0, 0, A_lb.data(), 0, nWSR);

  if (status != qpOASES::SUCCESSFUL_RETURN) {
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Infeasible");
    return q;
  }
  Eigen::Vector<real_t, 3> u;
  min_problem.getPrimalSolution(u.data());

  Eigen::Vector3d omega_opt(u(0), u(1), u(2));

  Eigen::Vector3d angle = omega_opt;  // * dt;
  // Eigen::Vector3d angle = omega; //* dt;
  Eigen::Matrix3d R_opt = R * exp_map(angle);

  // Compute the angles between the corresponding axes
  // Eigen::Vector3d angles;
  // for (int i = 0; i < 3; ++i) {
  //   double dot_product = R_ref.col(i).dot(R_opt.col(i));
  //   dot_product = std::clamp(dot_product, -1.0,
  //                            1.0);       // Ensure within valid range for
  //   angles(i) = std::acos(dot_product);  // Compute angle in radians
  // }
  // // Print the angles in radians
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
  //             "Angles between corresponding axes: x: %f | y: %f | z: %f",
  //             angles(0), angles(1), angles(2));

  Eigen::Quaterniond q_opt(R_opt);
  q_opt.normalize();
  return q_opt;
}

}  // namespace conic_cbf
#endif  // CONIC_CBF_QP_HPP
