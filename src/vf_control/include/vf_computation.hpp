#ifndef VF_COMPUTATION_HPP
#define VF_COMPUTATION_HPP
#define DEBUG true
#include "qp_wrapper.hpp"

namespace compute_vf
{
  template <typename Scalar>
  inline bool AlmostEqual(const Eigen::Matrix<Scalar, 3, 1> &vec1,
                          const Eigen::Matrix<Scalar, 3, 1> &vec2,
                          Scalar tolerance = static_cast<Scalar>(1e-6))
  {
    // Calculate the angle between the two vectors
    Scalar dotProduct = vec1.normalized().dot(vec2.normalized());
    Scalar angle = std::acos(dotProduct);

    // Calculate the axis difference (check if the vectors are almost parallel)
    bool axisAlmostEqual = vec1.normalized().isApprox(vec2.normalized(), tolerance);

    return ((angle <= tolerance) && axisAlmostEqual);
  }

  Eigen::Vector3d enforce_virtual_fixture(
      Mesh &mesh, const Eigen::Vector3d &target_position,
      const Eigen::Vector3d &current_position, const double radius,
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
          &constraint_planes,
      double lookup_area, Visualizer &vis)
  {

    // Find nearby triangles within a specified distance
    const double max_distance = radius + lookup_area;
    std::vector<int> T;
    mesh.find_nearby_triangles(current_position, max_distance, T);

    // std::cout << "Nearby triangles: " << T.size() << std::endl;
    if (T.size() == 0)
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
    for (int Ti : T)
    {
      CP[Ti] = mesh.get_closest_on_triangle(current_position, Ti);
    }
    // vis.DrawClosestPoints(CP, 0.002);
    int iteration = 0;
    for (auto it = T.begin(); it != T.end();)
    {
      // std::cout << "-----\nIteration: " << iteration++ << std::endl;
      int Ti = *it;
      auto Ni = mesh.normals[*it];
      Ni.normalize();
      auto [CPi, cpi_loc] = CP.at(*it);
      if (cpi_loc == Location::IN)
      {
        if (Ni.dot((current_position - CPi)) >= -eps)
        {
          // Constraint plane is the tangent plane to the sphere
          constraint_planes.push_back({Ni, CPi});
          it++;
          continue;
        }
      }
      else
      {
        if (cpi_loc == Location::V1 || cpi_loc == Location::V2 || cpi_loc == Location::V3 || cpi_loc == Location::VOID)
        {
          // Constraint plane is the tangent plane to the sphere
          if (AlmostEqual(current_position, CPi, eps))
          {
            Eigen::Vector3d n = mesh.normals[*it].normalized();
            for (auto j = it + 1; j != T.end();)
            {
              auto [CPj, cpj_loc] = CP.at(*j);
              if (AlmostEqual(CPi, CPj, eps) && mesh.is_locally_concave(*it, *j, cpi_loc))
              {
                n += mesh.normals[*j].normalized();
                T.erase(j);
              }
              else
              {
                j++;
              }
            }
            n.normalize();
            constraint_planes.push_back({n, CPi});
            it++;
            continue;
          }
          else
          {
            std::vector<int> neighborIdxList1, neighborIdxList2;
            if (cpi_loc == Location::V1)
            {
              neighborIdxList1 = mesh.adjacency_dict.at({Ti, Location::V1V2});
              neighborIdxList2 = mesh.adjacency_dict.at({Ti, Location::V1V3});
            }
            else if (cpi_loc == Location::V2)
            {
              neighborIdxList1 = mesh.adjacency_dict.at({Ti, Location::V1V2});
              neighborIdxList2 = mesh.adjacency_dict.at({Ti, Location::V2V3});
            }
            else if (cpi_loc == Location::V3)
            {
              neighborIdxList1 = mesh.adjacency_dict.at({Ti, Location::V1V3});
              neighborIdxList2 = mesh.adjacency_dict.at({Ti, Location::V2V3});
            }
            bool keep = false;
            int neighborIdx1, neighborIdx2;
            // check if neighbor is already in the list
            if (std::find(T.begin(), T.end(), neighborIdx1) == T.end())
            {
              CP[neighborIdx1] = mesh.get_closest_on_triangle(current_position, neighborIdx1);
            }
            if (std::find(T.begin(), T.end(), neighborIdx2) == T.end())
            {
              CP[neighborIdx2] = mesh.get_closest_on_triangle(current_position, neighborIdx2);
            }
            auto [CPn1, cpn1_loc] = CP.at(neighborIdx1);
            auto [CPn2, cpn2_loc] = CP.at(neighborIdx2);

            if (neighborIdxList1.size() > 0)
            {
              neighborIdx1 = neighborIdxList1[0];
              if (AlmostEqual(CPi, CPn1, eps))
              {
                for (auto j = it + 1; j != T.end();)
                {
                  if (*j == neighborIdx1)
                  {
                    T.erase(j);
                    CP[neighborIdx1].second = Location::VOID;
                    keep = true;
                    break;
                  }
                  else
                  {
                    j++;
                  }
                }
              }
            }
            if (neighborIdxList2.size() > 0)
            {
              neighborIdx2 = neighborIdxList2[0];
              if (AlmostEqual(CPi, CPn2, eps))
              {
                for (auto j = it + 1; j != T.end();)
                {
                  if (*j == neighborIdx2)
                  {
                    T.erase(j);
                    CP[neighborIdx2].second = Location::VOID;
                    keep = true;
                    break;
                  }
                  else
                  {
                    j++;
                  }
                }
              }
            }
            if (keep)
            {
              Eigen::Vector3d n = (current_position - CPi).normalized();
              constraint_planes.push_back({n, CPi});
              it++;
              continue;
            }
          }
        }
        else
        {
          // on edge
          auto neighborIdxList = mesh.adjacency_dict.at({Ti, cpi_loc});
          int neighborIdx;
          if (neighborIdxList.size() > 0)
          {
            neighborIdx = neighborIdxList[0];
            if (CP.find(neighborIdx) == CP.end())
            {
              CP[neighborIdx] = mesh.get_closest_on_triangle(current_position, neighborIdx);
            }
            auto [CPia, cpia_loc] = CP.at(neighborIdx);
            if (mesh.is_locally_concave(Ti, neighborIdx, cpi_loc))
            {
              if (AlmostEqual(CPia, CPi, eps))
              {
                if (AlmostEqual(current_position, CPi, eps))
                {
                  Eigen::Vector3d n = (Ni + mesh.normals[neighborIdx].normalized()).normalized();
                  constraint_planes.push_back({n, CPi});
                  it++;
                  continue;
                }
                else
                {
                  CP[neighborIdx].second = Location::VOID;
                  for (auto j = it + 1; j != T.end();)
                  {
                    if (*j == neighborIdx)
                    {
                      T.erase(j);
                      break;
                    }
                    else
                    {
                      j++;
                    }
                  }
                  Eigen::Vector3d n = (current_position - CPi).normalized();
                  constraint_planes.push_back({n, CPi});
                  it++;
                  continue;
                }
              }
            }
            else if (cpia_loc != Location::VOID && Ni.transpose().dot((current_position - CPi)) >= -eps)
            {
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

    int n_constraints = constraint_planes.size();
    // std::cout << "Found " << n_constraints << " constraints" << std::endl;

    // QP
    // solves argmin x^T H x + g^T x
    //        s.t. A x >= A_lb

    // hessian matrix
    const Eigen::Matrix<real_t, 3, 3, Eigen::RowMajor> H =
        Eigen::Matrix<real_t, 3, 3, Eigen::RowMajor>::Identity();
    // gradient vector
    auto direction = target_position - current_position;
    double step_size = std::min(direction.norm(), radius / 10);
    Eigen::Vector3d delta_x_des = direction.normalized() * step_size;
    Eigen::Vector<real_t, 3> g = -2 * (delta_x_des);
    // constraint matrix
    Eigen::Vector<real_t, Eigen::Dynamic> A_lb(n_constraints);

    Eigen::Matrix<real_t, Eigen::Dynamic, 3, Eigen::RowMajor> A(n_constraints, 3);
    for (int i = 0; i < n_constraints; ++i)
    {
      const Eigen::Vector3d n = constraint_planes[i].first;
      const Eigen::Vector3d p = constraint_planes[i].second;
      A_lb(i) = -n.transpose() * (current_position - p) + eps;
      A(i, 0) = n[0];
      A(i, 1) = n[1];
      A(i, 2) = n[2];
    }
    qpOASES::Options myOptions;
    myOptions.printLevel = qpOASES::PL_LOW;
    qpOASES::QProblem min_problem(3, n_constraints);
    min_problem.setOptions(myOptions);
    Eigen::Vector<real_t, 3> delta_x;
    int nWSR = 200;
    min_problem.init(H.data(), g.data(), A.data(), 0, 0, A_lb.data(), 0, nWSR);
    min_problem.getPrimalSolution(delta_x.data());
    // // std::cout << "delta_x: " << delta_x << std::endl;


    return delta_x.cast<double>();
  }
} // namespace compute_vf
#endif // VF_COMPUTATION_HPP