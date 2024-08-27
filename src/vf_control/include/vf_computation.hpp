#ifndef VF_COMPUTATION_HPP
#define VF_COMPUTATION_HPP
#define DEBUG true
#include "mesh.hpp"
#include "qp_wrapper.hpp"

Eigen::Vector3d enforce_virtual_fixture(
    Mesh &mesh, const Eigen::Vector3d &target_position,
    const Eigen::Vector3d &current_position, const double radius,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        &constraint_planes, double lookup_area)
{

  // Find nearby triangles within a specified distance
  const double max_distance = radius + lookup_area;
  std::vector<int> nearby_triangles =
      mesh.find_nearby_triangles(current_position, max_distance);

  if (nearby_triangles.size() == 0)
  {
    // No nearby triangles found; return the target position
    return target_position - current_position;
  }
  // std::cout << "Nearby triangles: " << nearby_triangles.size() << std::endl;

  // Construct constraints based on nearby triangles
  constraint_planes.clear();

  const double eps = 0.001;

  // Precompute CPi for all triangles
  std::unordered_map<int, std::pair<Eigen::Vector3d, Location>> CP;
  for (int Ti : nearby_triangles)
  {
    CP[Ti] = mesh.get_closest_on_triangle(current_position, Ti);
  }
  auto T = nearby_triangles;

  for (auto it = T.begin(); it != T.end();)
  {
    // std::cout <<"---------"<<std::endl;
    int Ti = *it;
    auto [CPi, cpi_loc] = CP.at(Ti);
    Eigen::Vector3d Ni = mesh.normals[Ti];
    auto face_center = (mesh.vertices[mesh.faces[Ti][0]] +
                        mesh.vertices[mesh.faces[Ti][1]] +
                        mesh.vertices[mesh.faces[Ti][2]]) /
                       3;
    Ni.normalize();
    if (Ni.dot(current_position - face_center) < 0)
    {
      Ni = -Ni;
    }

    // std::cout << "face " << Ti << std::endl;
    if (cpi_loc == Location::IN)
    {
      if (Ni.dot(current_position - CPi) >= -eps)
      {
        // std::cout << "IN -> adding"<<std::endl;
        constraint_planes.emplace_back(Ni, CPi);
        ++it; // Keep this triangle, move to the next
        continue;
      }
      else
      {
        // CP.at(Ti).second = Location::VOID;
        // T.erase(it);  // Remove this triangle
      }
    }
    else if (cpi_loc == Location::V1 || cpi_loc == Location::V2 ||
             cpi_loc == Location::V3)
    {
      // std::cout << "vertex"<<std::endl;
      // Handle vertex case
      if ((CPi - current_position).norm() < eps)
      {
        Eigen::Vector3d n = Ni;
        for (auto face_it = it + 1; face_it != T.end();)
        {
          if ((CP.at(*face_it).first - CPi).norm() < eps && mesh.is_locally_concave(Ti, *face_it, cpi_loc))
          {
            n += mesh.normals[*face_it].normalized();
            T.erase(face_it); // Remove current triangle
          }
          else
          {
            ++face_it;
          }
        }
        n.normalize();
        constraint_planes.emplace_back(n, CPi);
        ++it;
        continue;
      }
      else
      {
        std::vector<int> neighborIdx1, neighborIdx2;
        if (cpi_loc == Location::V1)
        {
          neighborIdx1 = mesh.adjacency_dict.at({Ti, Location::V1V2});
          neighborIdx2 = mesh.adjacency_dict.at({Ti, Location::V1V3});
        }
        else if (cpi_loc == Location::V2)
        {
          neighborIdx1 = mesh.adjacency_dict.at({Ti, Location::V1V2});
          neighborIdx2 = mesh.adjacency_dict.at({Ti, Location::V2V3});
        }
        else if (cpi_loc == Location::V3)
        {
          neighborIdx1 = mesh.adjacency_dict.at({Ti, Location::V1V3});
          neighborIdx2 = mesh.adjacency_dict.at({Ti, Location::V2V3});
        }

        bool keep = false;
        // std::cout << "neighbor1: " << neighborIdx1.size() << std::endl;
        // std::cout << "neighbor2: " << neighborIdx2.size() << std::endl;
        // Check both neighbor indices
        for (const auto &neighborIdx : {neighborIdx1, neighborIdx2})
        {
          if (!neighborIdx.empty())
          {
            // std::cout << "neighbor"<<std::endl;
            int neighbor = neighborIdx[0];

            // Ensure CP exists for the neighbor
            if (CP.find(neighbor) == CP.end())
            {
              // std::cout << "CP not found -> computing"<<std::endl;
              CP[neighbor] =
                  mesh.get_closest_on_triangle(current_position, neighbor);
            }
            auto [CPia, _] = CP.at(neighbor);
            if ((CPia - CPi).norm() < eps)
            {
              for (auto face_it = it + 1; face_it != T.end();)
              {
                if (*face_it == neighbor)
                {
                  T.erase(face_it); // Remove neighbor triangle
                  CP.at(neighbor).second = Location::VOID;
                  keep = true;
                  break;
                }
                else
                {
                  ++face_it;
                }
              }
            }
          }
        }

        if (keep)
        {
          // std::cout << "adding vertex" << std::endl;
          Eigen::Vector3d n = (current_position - CPi).normalized();
          constraint_planes.emplace_back(n, CPi);
          ++it; // Keep current triangle, move to the next
          continue;
        }
      }
      // else {
      //   // std::cout << "removing vertex" << std::endl;
      //   CP.at(Ti).second = Location::VOID;
      //   T.erase(it);  // Remove current triangle
      // }
    }
    else if (cpi_loc != Location::VOID)
    {
      // Handle edge case
      std::vector<int> neighborIdx = mesh.adjacency_dict.at({Ti, cpi_loc});
      if (!neighborIdx.empty())
      {
        int neighborIdx0 = neighborIdx[0];

        if (CP.find(neighborIdx0) == CP.end())
        {
          CP[neighborIdx0] =
              mesh.get_closest_on_triangle(current_position, neighborIdx0);
        }

        auto [CPia, cpia_loc] = CP.at(neighborIdx0);
        if (mesh.is_locally_concave(Ti, neighborIdx0, cpi_loc))
        {
          if ((CPi - CPia).norm() < eps)
          {
            if ((current_position - CPi).norm() < eps)
            {
              // average normals for stability
              Eigen::Vector3d n = (Ni + mesh.normals[neighborIdx0].normalized()).normalized();
              constraint_planes.emplace_back(n, CPi);
              it++;
              continue;
            }
            else
            {
              CP.at(neighborIdx0).second = Location::VOID;
              for (auto face_it = it + 1; face_it != T.end();)
              {
                if (*face_it == neighborIdx0)
                {
                  T.erase(face_it); // Remove neighbor triangle
                  break;
                }
                else
                {
                  ++face_it;
                }
              }
              Eigen::Vector3d n = (current_position - CPi).normalized();
              constraint_planes.emplace_back(n, CPi);
              ++it; // Keep current triangle, move to the next
              continue;
            }
          }
        }
        else if (cpia_loc != Location::VOID &&
                 Ni.dot(current_position - CPi) >= -eps)
        {
          constraint_planes.emplace_back(Ni, CPi);
          ++it; // Keep current triangle, move to the next
          continue;
        }
      }
    }

    // If none of the above cases, remove the current triangle
    // std::cout <<"removing"<<std::endl;
    CP.at(Ti).second = Location::VOID;
    T.erase(it); // Remove current triangle
  }

  int n_constraints = constraint_planes.size();
  // std::cout << "Number of constraints: " << n_constraints << std::endl;

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
  for (int i = 0; i < n_constraints; ++i)
  {
    const Eigen::Vector3d n = constraint_planes[i].first;
    const Eigen::Vector3d p = constraint_planes[i].second;
    A_lb(i) = -n.transpose() * (current_position - p) + radius;
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
  // // std::cout << "delta_x: " << delta_x << std::endl;
  return delta_x.cast<double>();
}
#endif // VF_COMPUTATION_HPP