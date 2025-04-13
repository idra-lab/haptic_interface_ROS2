#ifndef VF_COMPUTATION_HPP
#define VF_COMPUTATION_HPP
#define DEBUG true
#include "qp_wrapper.hpp"
#include "utils/visualization.hpp"

namespace compute_vf {
template <typename Scalar>
inline bool almost_equal(const Eigen::Matrix<Scalar, 3, 1> &vec1,
                         const Eigen::Matrix<Scalar, 3, 1> &vec2,
                         Scalar tolerance = static_cast<Scalar>(1e-6)) {
  // Calculate the angle between the two vectors
  Scalar dotProduct = vec1.normalized().dot(vec2.normalized());
  Scalar angle = std::acos(dotProduct);

  // Calculate the axis difference (check if the vectors are almost parallel)
  bool axisalmost_equal =
      vec1.normalized().isApprox(vec2.normalized(), tolerance);

  return ((angle <= tolerance) && axisalmost_equal);
}

Eigen::Vector3d enforce_virtual_fixture(
    Mesh &mesh, const Eigen::Vector3d &target_position,
    const Eigen::Vector3d &current_position, const double radius,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &constraint_planes,
    double lookup_area, Visualizer &vis) {
  (void)vis;
  // Find nearby triangles within a specified distance
  const double max_distance = radius + lookup_area;
  std::vector<int> T;
  mesh.find_nearby_triangles(current_position, max_distance, T);

  // std::cout << "Nearby triangles: " << T.size() << std::endl;

  // Construct constraints based on nearby triangles
  constraint_planes.clear();

  const double eps = 0.001;

  // Precompute CPi for all triangles
  std::unordered_map<int, std::pair<Eigen::Vector3d, Location>> CP;
  for (int Ti : T) {
    CP[Ti] = mesh.get_closest_on_triangle(current_position, Ti);
  }
  // vis.draw_closest_points(CP, 0.002);
  // int iter = 0;
  for (auto it = T.begin(); it != T.end();) {
    // std::cout << iter++ << std::endl;
    int Ti = *it;
    auto Ni = mesh.normals[*it];
    Ni.normalize();
    auto [CPi, cpi_loc] = CP.at(*it);
    if (cpi_loc == Location::IN) {
      if (Ni.dot((current_position - CPi)) >= -eps) {
        // Constraint plane is the tangent plane to the sphere
        constraint_planes.push_back({Ni, CPi});
        it++;
        continue;
      }
    } else {
      if (cpi_loc == Location::V1 || cpi_loc == Location::V2 ||
          cpi_loc == Location::V3 || cpi_loc == Location::VOID) {
        // Constraint plane is the tangent plane to the sphere
        if (almost_equal(current_position, CPi, eps)) {
          Eigen::Vector3d n = mesh.normals[*it].normalized();
          for (auto j = it + 1; j != T.end();) {
            auto [CPj, cpj_loc] = CP.at(*j);
            if (almost_equal(CPi, CPj, eps) &&
                mesh.is_locally_concave(*it, *j, cpi_loc)) {
              n += mesh.normals[*j].normalized();
              T.erase(j);
            } else {
              j++;
            }
          }
          n.normalize();
          constraint_planes.push_back({n, CPi});
          it++;
          continue;
        } else {
          std::vector<int> neighborIdxList1, neighborIdxList2;
          if (cpi_loc == Location::V1) {
            neighborIdxList1 = mesh.adjacency_dict.at({Ti, Location::V1V2});
            neighborIdxList2 = mesh.adjacency_dict.at({Ti, Location::V1V3});
          } else if (cpi_loc == Location::V2) {
            neighborIdxList1 = mesh.adjacency_dict.at({Ti, Location::V1V2});
            neighborIdxList2 = mesh.adjacency_dict.at({Ti, Location::V2V3});
          } else if (cpi_loc == Location::V3) {
            neighborIdxList1 = mesh.adjacency_dict.at({Ti, Location::V1V3});
            neighborIdxList2 = mesh.adjacency_dict.at({Ti, Location::V2V3});
          } else {
            it++;
            continue;
          }
          bool keep = false;
          if (neighborIdxList1.size() == 0 || neighborIdxList2.size() == 0) {
            it++;
            continue;
          }
          int neighborIdx1 = neighborIdxList1[0];
          int neighborIdx2 = neighborIdxList2[0];
          // check if neighbor is already in the list otherwise add it
          if (std::find(T.begin(), T.end(), neighborIdx1) == T.end()) {
            CP[neighborIdx1] =
                mesh.get_closest_on_triangle(current_position, neighborIdx1);
          }
          if (std::find(T.begin(), T.end(), neighborIdx2) == T.end()) {
            CP[neighborIdx2] =
                mesh.get_closest_on_triangle(current_position, neighborIdx2);
          }
          auto [CPn1, cpn1_loc] = CP.at(neighborIdx1);
          auto [CPn2, cpn2_loc] = CP.at(neighborIdx2);

          if (almost_equal(CPi, CPn1, eps)) {
            for (auto j = it + 1; j != T.end();) {
              if (*j == neighborIdx1) {
                T.erase(j);
                CP[neighborIdx1].second = Location::VOID;
                keep = true;
                break;
              } else {
                j++;
              }
            }
          }
          if (almost_equal(CPi, CPn2, eps)) {
            for (auto j = it + 1; j != T.end();) {
              if (*j == neighborIdx2) {
                T.erase(j);
                CP[neighborIdx2].second = Location::VOID;
                keep = true;
                break;
              } else {
                j++;
              }
            }
          }
          if (keep) {
            Eigen::Vector3d n = (current_position - CPi).normalized();
            constraint_planes.push_back({n, CPi});
            it++;
            continue;
          }
        }
      } else {
        // on edge
        auto neighborIdxList = mesh.adjacency_dict.at({Ti, cpi_loc});
        int neighborIdx;
        if (neighborIdxList.size() > 0) {
          neighborIdx = neighborIdxList[0];
          if (CP.find(neighborIdx) == CP.end()) {
            CP[neighborIdx] =
                mesh.get_closest_on_triangle(current_position, neighborIdx);
          }
          auto [CPia, cpia_loc] = CP.at(neighborIdx);
          if (mesh.is_locally_concave(Ti, neighborIdx, cpi_loc)) {
            if (almost_equal(CPia, CPi, eps)) {
              if (almost_equal(current_position, CPi, eps)) {
                Eigen::Vector3d n =
                    (Ni + mesh.normals[neighborIdx].normalized()).normalized();
                constraint_planes.push_back({n, CPi});
                it++;
                continue;
              } else {
                CP[neighborIdx].second = Location::VOID;
                for (auto j = it + 1; j != T.end();) {
                  if (*j == neighborIdx) {
                    T.erase(j);
                    break;
                  } else {
                    j++;
                  }
                }
                Eigen::Vector3d n = (current_position - CPi).normalized();
                constraint_planes.push_back({n, CPi});
                it++;
                continue;
              }
            }
          } else if (cpia_loc != Location::VOID &&
                     Ni.transpose().dot((current_position - CPi)) >= -eps) {
            constraint_planes.push_back({Ni, CPi});
            it++;
            continue;
          }
        }
      }
    }
    CP[*it].second = Location::VOID;
    T.erase(it);
  }

  const int n_constraints = constraint_planes.size();
  // std::cout << "Found " << n_constraints << " constraints" << std::endl;
  Eigen::Vector3d delta_x_des = target_position - current_position;
  if (n_constraints == 0) {
    return delta_x_des;
  } else {
    auto direction = target_position - current_position;
    double step_size = std::min(direction.norm(), radius / 10);
    delta_x_des = direction.normalized() * step_size;
  }

  // QP
  // solves argmin x^T H x + g^T x
  //        s.t. A x >= A_lb

  // gradient vector
  Eigen::Vector<real_t, 3> g = -delta_x_des;
  // constraint matrix
  Eigen::Vector<real_t, Eigen::Dynamic> A_lb(n_constraints);  //,lb(3),ub(3);

  Eigen::Vector<real_t, 3> lb(3);

  Eigen::Matrix<real_t, Eigen::Dynamic, 3, Eigen::RowMajor> A(n_constraints, 3);
  for (int i = 0; i < n_constraints; ++i) {
    const Eigen::Vector3d n = constraint_planes[i].first;
    const Eigen::Vector3d p = constraint_planes[i].second;
    A_lb(i) = -n.transpose() * (current_position - p) +
              radius;  // ultrasound length = 0.23 mm
    A(i, 0) = n[0];
    A(i, 1) = n[1];
    A(i, 2) = n[2];
  }

  // Assuming x is of dimension 3
  const float max_delta = 0.0001;
  // adding constraint on max delta to avoid oscillations between two points
  real_t x_lb[3] = {-max_delta, -max_delta, -max_delta};  // lower bound
  real_t x_ub[3] = {max_delta, max_delta, max_delta};     // upper bound
  qpOASES::Options myOptions;
  myOptions.printLevel = qpOASES::PL_LOW;
  qpOASES::QProblem min_problem(3, n_constraints,
                                qpOASES::HessianType::HST_IDENTITY);
  min_problem.setOptions(myOptions);
  Eigen::Vector<real_t, 3> delta_x;
  int nWSR = 200;
  if (n_constraints != 0) {
    min_problem.init(0, g.data(), A.data(), x_lb, x_ub, A_lb.data(), 0, nWSR);
  }
  // catch no solution
  if (min_problem.isInitialised() == false) {
    std::cerr << "QP problem not initialized" << std::endl;
    return Eigen::Vector3d::Zero();
  }
  auto success = min_problem.getPrimalSolution(delta_x.data());
  if (success != qpOASES::SUCCESSFUL_RETURN) {
    std::cerr << "QP problem not solved" << std::endl;
    return Eigen::Vector3d::Zero();
  }
  // std::cout << "delta_x: " << delta_x << std::endl;

  return delta_x.cast<double>();
}
}  // namespace compute_vf
#endif  // VF_COMPUTATION_HPP