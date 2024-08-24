#ifndef VF_COMPUTATION_HPP
#define VF_COMPUTATION_HPP

#include "mesh.hpp"
#include "qp_wrapper.hpp"

void computePlanes(
    Mesh &mesh, const Eigen::Vector3d &target_position,
    const Eigen::Vector3d &current_position,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &constraint_planes,
    std::unordered_map<int, std::pair<Eigen::Vector3d, Location>> &CP,
    std::vector<int> &T) {
  const double eps = 0.001;
  for (auto it = T.begin(); it != T.end(); ++it) {
    std::cout << "------\n" << "Iteration: " << it - T.begin() << std::endl;
    int Ti = *it;
    std::cout << "Triangle: " << Ti << std::endl;
    auto [CPi, cpi_loc] = CP.at(Ti);
    std::cout << "CPi: " << CPi << std::endl;
    Eigen::Vector3d Ni = mesh.normals[Ti];
    Ni.normalize();

    if (cpi_loc == Location::IN && Ni.dot(current_position - CPi) >= -eps) {
      constraint_planes.push_back({Ni, CPi});
      std::cout << "IN" << std::endl;
      continue;
    } else {
      if (cpi_loc != Location::VOID && cpi_loc != Location::IN) {
        // find adjacent
        std::cout << "Retrieving adjacent faces of " << Ti << " at location "
                  << LocationToString(cpi_loc) << std::endl;

        std::vector<int> neighbors = mesh.adjacency_dict.at({Ti, cpi_loc});
        for (auto neighbor : neighbors) {
          std::cout << "--- Neighbor: " << neighbor << std::endl;
          if (CP.find(neighbor) == CP.end()) {
            std::cout << "Was not in CP -> computing" << std::endl;
            CP[neighbor] =
                mesh.get_closest_on_triangle(current_position, neighbor);
          }
          std::cout << "Reading CP" << std::endl;
          auto [CPia, _] = CP.at(neighbor);
          if ((CPi - CPia).norm() < eps &&
              mesh.is_locally_convex(Ti, neighbor, cpi_loc)) {
            Eigen::Vector3d n = (current_position - CPi).normalized();
            constraint_planes.push_back({n, CPi});
            std::cout << "EDGE CASE 1" << std::endl;
            continue;
          }
          if (Ni.dot(current_position - CPi) >= -eps &&
              mesh.is_locally_concave(Ti, neighbor, cpi_loc)) {
            constraint_planes.push_back({Ni, CPi});
            std::cout << "EDGE CASE 2" << std::endl;
            continue;
          }
        }
      }
    }
  }
  std::cout << "Found " << constraint_planes.size() << " planes" << std::endl;
}

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

  if (nearby_triangles.empty()) {
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

  std::vector<int> T =
      nearby_triangles;  // Directly use a std::vector for easier modification
  int iteration = 0;

  // computePlanes(mesh, target_position, current_position, constraint_planes,
  // CP,
  //               T);
  // Process constraints
  for (auto it = T.begin(); it != T.end();) {
    iteration++;
    int Ti = *it;
    auto [CPi, cpi_loc] = CP.at(Ti);
    Eigen::Vector3d Ni = mesh.normals[Ti];
    Ni.normalize();
    // LOCATION IN //
    if (cpi_loc == Location::IN) {
      if (Ni.dot(current_position - CPi) >= -eps) {
        constraint_planes.push_back({Ni, CPi});
        ++it;  // Keep current triangle, move to the next
      } else {
        CP.at(Ti).second = Location::VOID;
        it = T.erase(it);  // Remove current triangle
      }
    } 
    else 
    {  // LOCATION ON EDGE OR VERTEX //
          
        // LOCATION ON VERTEX //
      if (cpi_loc == Location::V1 || cpi_loc == Location::V2 ||
          cpi_loc == Location::V3) {
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
          if (CP.find(neighborIdx) == CP.end()) {
            CP[neighborIdx] =
                mesh.get_closest_on_triangle(current_position, neighborIdx);
          }
          auto [CPia, _] = CP.at(neighborIdx);
          if ((CPia - CPi).norm() < eps) {
            for (auto face_it = it + 1; face_it != T.end();) {
              if (*face_it == neighborIdx) {
                T.erase(face_it);
                CP.at(neighborIdx).second = Location::VOID;
                keep = true;
                break;
              } else {
                ++face_it;
              }
            }
          }
        }

        if (neighborIdx2.size() > 0) {
          int neighborIdx = neighborIdx2[0];
          if (CP.find(neighborIdx) == CP.end()) {
            CP[neighborIdx] =
                mesh.get_closest_on_triangle(current_position, neighborIdx);
          }
          auto [CPia, _] = CP.at(neighborIdx);
          if ((CPia - CPi).norm() < eps) {
            for (auto face_it = it + 1; face_it != T.end();) {
              if (*face_it == neighborIdx) {
                T.erase(face_it);
                CP.at(neighborIdx).second = Location::VOID;
                keep = true;
                break;
              } else {
                ++face_it;
              }
            }
          }
        }

        if (keep) {
          Eigen::Vector3d n = (current_position - CPi).normalized();
          constraint_planes.push_back({n, CPi});
          ++it;  // Keep current triangle, move to the next
        } else {
          CP.at(Ti).second = Location::VOID;
          it = T.erase(it);  // Remove current triangle
        }
      } else {
        // LOCATION ON EDGE //
        if (cpi_loc != Location::VOID) {
          std::vector<int> neighborIdx = mesh.adjacency_dict.at({Ti, cpi_loc});
          if (!neighborIdx.empty()) {
            int neighborIdx0 = neighborIdx[0];
            if (CP.find(neighborIdx0) == CP.end()) {
              CP[neighborIdx0] =
                  mesh.get_closest_on_triangle(current_position, neighborIdx0);
            }
            auto [CPia, cpia_loc] = CP.at(neighborIdx0);
            if (mesh.is_locally_concave(Ti, neighborIdx0, cpi_loc)) {
              if ((CPi - CPia).norm() < eps) {
                CP.at(neighborIdx0).second = Location::VOID;
                for (auto face_it = it + 1; face_it != T.end();) {
                  if (*face_it == neighborIdx0) {
                    T.erase(face_it);
                    break;
                  } else {
                    ++face_it;
                  }
                }
                Eigen::Vector3d n = (current_position - CPi).normalized();
                constraint_planes.push_back({n, CPi});
                ++it;  // Keep current triangle, move to the next
              } else {
                CP.at(Ti).second = Location::VOID;
                it = T.erase(it);  // Remove current triangle
              }
            } else {
              if (cpia_loc != Location::VOID &&
                  Ni.dot(current_position - CPi) >= -eps) {
                constraint_planes.push_back({Ni, CPi});
                ++it;  // Keep current triangle, move to the next
              } else {
                CP.at(Ti).second = Location::VOID;
                it = T.erase(it);  // Remove current triangle
              }
            }
          } else {
            CP.at(Ti).second = Location::VOID;
            it = T.erase(it);  // Remove current triangle
          }
        } else {
          CP.at(Ti).second = Location::VOID;
          it = T.erase(it);  // Remove current triangle
        }
      }
    }
  }

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
  int nWSR = 10;
  min_problem.init(H.data(), g.data(), A.data(), 0, 0, A_lb.data(), 0, nWSR);
  min_problem.getPrimalSolution(delta_x.data());
  // std::cout << "delta_x: " << delta_x << std::endl;
  return delta_x.cast<double>();
}
#endif  // VF_COMPUTATION_HPP