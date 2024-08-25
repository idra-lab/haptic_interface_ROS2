#ifndef VF_COMPUTATION_HPP
#define VF_COMPUTATION_HPP

#include "mesh.hpp"
#include "qp_wrapper.hpp"

Eigen::Vector3d enforce_virtual_fixture(
    Mesh &mesh, const Eigen::Vector3d &target_position,
    const Eigen::Vector3d &current_position, const double radius,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        &constraint_planes) {
  // Define a small buffer distance to prevent penetration
  const double lookup_area = radius;

  // Find nearby triangles within a specified distance
  const double max_distance = radius + lookup_area;
  std::vector<int> nearby_triangles =
      mesh.find_nearby_triangles(current_position, max_distance);

  if (nearby_triangles.size() == 0) {
    // No nearby triangles found; return the target position
    return target_position - current_position;
  }
  std::cout << "Nearby triangles: " << nearby_triangles.size() << std::endl;

  // Construct constraints based on nearby triangles
  constraint_planes.clear();

  const double eps = 0.001;

  // Precompute CPi for all triangles
  std::unordered_map<int, std::pair<Eigen::Vector3d, Location>> CP;
  for (int Ti : nearby_triangles) {
    CP[Ti] = mesh.get_closest_on_triangle(current_position, Ti);
  }
  auto T = nearby_triangles;

  for (size_t i = 0; i < T.size(); ++i) {
    int Ti = T[i];

    // Find the closest point on the triangle to the sphere center
    auto [CPi, cpi_loc] = CP[Ti];

    // Normalize the triangle normal
    Eigen::Vector3d Ni = mesh.normals[Ti];
    Ni.normalize();

    // Check if CPi is in the triangle and the normal points towards the sphere
    // center
    if (cpi_loc == Location::IN) {
      if (Ni.transpose().dot(current_position - CPi) >= -eps) {
        constraint_planes.push_back({Ni, CPi});
        continue;
      }
    } else {
      // Handle points on the edges or vertices
      if (cpi_loc == Location::V1 || cpi_loc == Location::V2 ||
          cpi_loc == Location::V3) {
        Eigen::Vector3d n = Ni;

        if ((current_position - CPi).norm() < eps) {
          // Find all same vertex, average and remove
          for (size_t j = i + 1; j < T.size(); ++j) {
            int face = T[j];
            auto [CPia, _] = CP[face];

            if ((CPia - CPi).norm() < eps) {
              n += mesh.normals[face].normalized();
              auto it = std::find(T.begin(), T.end(), face);
              if (it != T.end()) T.erase(it);
              --j;  // Adjust index after deletion
            }
          }

          // Normalize normal
          n.normalize();
          constraint_planes.push_back({n, CPi});
          
          continue;
        } else {
          std::vector<int> neighborIdx1, neighborIdx2;

          if (cpi_loc == Location::V1) {
            neighborIdx1 = mesh.adjacency_dict.at({Ti, Location::V1V2});
            neighborIdx2 = mesh.adjacency_dict.at({Ti, Location::V1V3});
          } else if (cpi_loc == Location::V2) {
            neighborIdx1 = mesh.adjacency_dict.at({Ti, Location::V1V2});
            neighborIdx2 = mesh.adjacency_dict.at({Ti, Location::V2V3});
          } else if (cpi_loc == Location::V3) {
            neighborIdx1 = mesh.adjacency_dict.at({Ti, Location::V1V3});
            neighborIdx2 = mesh.adjacency_dict.at({Ti, Location::V2V3});
          } else {
            std::cerr << "Error: location not on vertex" << std::endl;
            exit(EXIT_FAILURE);
          }

          bool keep = false;

          if (neighborIdx1.size() > 0) {
            int neighborIdx = neighborIdx1[0];
            bool is_in_cp_list = true;
            auto CPia_iter = CP.find(neighborIdx);
            Eigen::Vector3d CPia;

            if (CPia_iter != CP.end()) {
              CPia = CPia_iter->second.first;
            } else {
              is_in_cp_list = false;
              std::tie(CPia, std::ignore) =
                  mesh.get_closest_on_triangle(current_position, neighborIdx);
            }

            if ((CPia - CPi).norm() < eps) {
              auto it = std::find(T.begin(), T.end(), neighborIdx);
              if (it != T.end()) T.erase(it);
              if (is_in_cp_list) CP[neighborIdx].second = Location::VOID;
              keep = true;
            }
          }

          if (!neighborIdx2.size() > 0) {
            int neighborIdx = neighborIdx2[0];
            bool is_in_cp_list = true;
            auto CPia_iter = CP.find(neighborIdx);
            Eigen::Vector3d CPia;

            if (CPia_iter != CP.end()) {
              CPia = CPia_iter->second.first;
            } else {
              is_in_cp_list = false;
              std::tie(CPia, std::ignore) =
                  mesh.get_closest_on_triangle(current_position, neighborIdx);
            }

            if ((CPia - CPi).norm() < eps) {
              auto it = std::find(T.begin(), T.end(), neighborIdx);
              if (it != T.end()) T.erase(it);
              if (is_in_cp_list) CP[neighborIdx].second = Location::VOID;
              keep = true;
            }
          }

          if (keep) {
            Eigen::Vector3d n = current_position - CPi;
            n.normalize();
            constraint_planes.push_back({n, CPi});
            
            continue;
          }
        }
      } else {
        if (cpi_loc != Location::VOID) {
          auto neighborIdxs = mesh.adjacency_dict.at({Ti, cpi_loc});

          if (neighborIdxs.size() > 0) {
            int neighborIdx = neighborIdxs[0];
            bool is_in_cp_list = true;
            auto CPia_iter = CP.find(neighborIdx);
            Eigen::Vector3d CPia;

            if (CPia_iter != CP.end()) {
              CPia = CPia_iter->second.first;
            } else {
              is_in_cp_list = false;
              std::tie(CPia, std::ignore) =
                  mesh.get_closest_on_triangle(current_position, neighborIdx);
            }

            if (mesh.is_locally_concave(Ti, neighborIdx, cpi_loc)) {
              if ((CPi - CPia).norm() < eps) {
                if ((current_position - CPi).norm() < eps) {
                  Eigen::Vector3d n = Ni + mesh.normals[neighborIdx].normalized();
                    n.normalize();
                  constraint_planes.push_back({n, CPi});
                  
                  continue;
                } else {
                  if (is_in_cp_list) CP[neighborIdx].second = Location::VOID;

                  auto it = std::find(T.begin(), T.end(), neighborIdx);
                  if (it != T.end()) T.erase(it);

                  Eigen::Vector3d n = current_position - CPi;
                  n.normalize();
                  constraint_planes.push_back({n, CPi});
                  
                  continue;
                }
              }
            } else if (cpi_loc != Location::VOID &&
                       Ni.transpose().dot(current_position - CPi) >= -eps) {
              constraint_planes.push_back({Ni, CPi});
              
              continue;
            }
          } else {
            continue;
          }
        }
      }
    }

    // otherwise, delete current mesh
    CP[Ti].second = Location::VOID;
    auto it = std::find(T.begin(), T.end(), Ti);
    if (it != T.end()) T.erase(it);
  }
  int iteration = 0;

  int n_constraints = constraint_planes.size();
  std::cout << "Number of constraints: " << n_constraints << std::endl;

  // QP
  // solves argmin x^T H x + g^T x
  //        s.t. A x >= A_lb

  // hessian matrix
  const Eigen::Matrix<real_t, 3, 3, Eigen::RowMajor> H =
      Eigen::Matrix<real_t, 3, 3, Eigen::RowMajor>::Identity();
  // gradient vector
  auto direction = target_position - current_position;
  double step_size = std::min(direction.norm(), radius);
  Eigen::Vector3d delta_x_des = direction.normalized() * step_size;
  Eigen::Vector<real_t, 3> g = -2 * (delta_x_des);
  // constraint matrix
  Eigen::Vector<real_t, Eigen::Dynamic> A_lb(n_constraints);

  Eigen::Matrix<real_t, Eigen::Dynamic, 3, Eigen::RowMajor> A(n_constraints, 3);
  for (int i = 0; i < n_constraints; ++i) {
    const Eigen::Vector3d n = constraint_planes[i].first;
    const Eigen::Vector3d p = constraint_planes[i].second;
    A_lb(i) = -n.transpose() * (current_position - p);
    A(i, 0) = n[0];
    A(i, 1) = n[1];
    A(i, 2) = n[2];
  }
  qpOASES::Options myOptions;
  myOptions.printLevel = qpOASES::PL_LOW;
  qpOASES::QProblem min_problem(3, n_constraints);
  min_problem.setOptions(myOptions);
  Eigen::Vector<real_t, 3> delta_x;
  int nWSR = 100;
  min_problem.init(H.data(), g.data(), A.data(), 0, 0, A_lb.data(), 0, nWSR);
  min_problem.getPrimalSolution(delta_x.data());
  // std::cout << "delta_x: " << delta_x << std::endl;
  return delta_x.cast<double>();
}
#endif  // VF_COMPUTATION_HPP