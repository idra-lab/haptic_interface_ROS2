#ifndef VF_COMPUTATION_HPP
#define VF_COMPUTATION_HPP

#include "mesh.hpp"
#include "qp_wrapper.hpp"

Eigen::Vector3d enforce_virtual_fixture(Mesh &mesh, const Eigen::Vector3d &target_position,
                                        const Eigen::Vector3d &current_position, const double sphere_radius,
                                        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& constraint_planes)
{
    // Define a small buffer distance to prevent penetration
    const double lookup_area = sphere_radius;

    // Find nearby triangles within a specified distance
    const double max_distance = sphere_radius + lookup_area;
    std::vector<int> nearby_triangles = mesh.find_nearby_triangles(current_position, max_distance);

    if (nearby_triangles.empty())
    {
        // No nearby triangles found; return the target position
        return target_position - current_position;
    }
    std::cout << "Nearby triangles: " << nearby_triangles.size() << std::endl;

    // Construct constraints based on nearby triangles
    constraint_planes.clear();

    const double eps = 0.001;

    // Precompute CPi for all triangles
    std::unordered_map<int, std::pair<Eigen::Vector3d, Location>> CP;
    for (int Ti : nearby_triangles)
    {
        CP[Ti] = mesh.get_closest_on_triangle(current_position, Ti);
    }

    std::vector<int> T = nearby_triangles; // Directly use a std::vector for easier modification

    // Process constraints
    for (auto it = T.begin(); it != T.end(); /* no increment here */)
    {
        int Ti = *it;
        auto [CPi, cpi_loc] = CP[Ti];
        Eigen::Vector3d Ni = mesh.normals[Ti];
        Ni.normalize();

        if (cpi_loc == Location::IN)
        {
            if (Ni.dot(current_position - CPi) >= 0)
            {
                constraint_planes.emplace_back(Ni, CPi);
                ++it;
                continue;
            }
        }
        else
        {
            // on vertex
            if (cpi_loc == Location::V1 || cpi_loc == Location::V2 || cpi_loc == Location::V3)
            {
                Eigen::Vector3d n = Ni;
                if ((current_position - CPi).norm() < eps)
                {
                    for (auto face_it = std::next(it); face_it != T.end(); /* no increment here */)
                    {
                        auto [CPia, _] = CP[*face_it];
                        if ((CPia - CPi).norm() < eps)
                        {
                            n += mesh.normals[*face_it].normalized();
                            // Remove face from T
                            face_it = T.erase(face_it);
                            if (face_it != T.end())
                            {
                                CP[*face_it].second = Location::VOID;
                            }
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
                // proceed as normal
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
                    else
                    {
                        std::cerr << "Error: location not on vertex" << std::endl;
                        exit(EXIT_FAILURE);
                    }

                    bool keep = false;
                    // if (closestPoint.at(*i).AlmostEqual(closestPoint.at(neighborIdx1), mMeshResolution))

                    for (auto neighborIdx : {neighborIdx1, neighborIdx2})
                    {
                        if (neighborIdx.empty())
                        {
                            continue;
                        }
                        int neighbor = neighborIdx[0];
                        auto [CPia, _] = CP[neighbor];
                        if ((CPia - CPi).norm() < eps)
                        {
                            // TODO: check for seg fault
                            for (auto face_it = std::next(it); face_it != T.end(); /* no increment here */)
                            {
                                if (*face_it == neighbor)
                                {
                                    // Remove neighbor T
                                    face_it = T.erase(face_it);
                                    if (face_it != T.end())
                                    {
                                        CP[*face_it].second = Location::VOID;
                                    }
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
                    if (keep)
                    {
                        Eigen::Vector3d n = (current_position - CPi).normalized();
                        constraint_planes.emplace_back(n, CPi);
                        ++it;
                        continue;
                    }
                }
            }
            else
            {
                // on edge
                if (cpi_loc != Location::VOID)
                {
                    std::vector<int> neighborIdx = mesh.adjacency_dict.at({Ti, cpi_loc});
                    if (!neighborIdx.empty())
                    {
                        int neighborIdx0 = neighborIdx[0];
                        // TODO: check for seg fault
                        auto [CPia, cpia_loc] = CP[neighborIdx0];
                        if (CPia == Eigen::Vector3d::Zero())
                        {
                            CPia = mesh.get_closest_on_triangle(current_position, neighborIdx0).first;
                        }
                        if (mesh.is_locally_concave(Ti, neighborIdx0, cpi_loc))
                        {
                            if ((CPi - CPia).norm() < eps)
                            {
                                if ((current_position - CPi).norm() < eps)
                                {
                                    Eigen::Vector3d n = Ni + mesh.normals[neighborIdx0].normalized();
                                    n.normalize();
                                    constraint_planes.emplace_back(n, CPi);
                                    ++it;
                                    continue;
                                }
                                else
                                {
                                    CP[neighborIdx0].second = Location::VOID;
                                    for (auto face_it = std::next(it); face_it != T.end(); /* no increment here */)
                                    {
                                        if (*face_it == neighborIdx0)
                                        {
                                            face_it = T.erase(face_it);
                                            if (face_it != T.end())
                                            {
                                                CP[*face_it].second = Location::VOID;
                                            }
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
                        else if (cpia_loc != Location::VOID && Ni.dot(current_position - CPi) >= 0)
                        {
                            constraint_planes.emplace_back(Ni, CPi);
                            ++it;
                            continue;
                        }
                    }
                }
            }
        }
        // Otherwise, remove current triangle
        CP[Ti].second = Location::VOID;
        it = T.erase(it);
    }

    int n_constraints = constraint_planes.size();
    std::cout << "Number of constraints: " << n_constraints << std::endl;
    
    // QP 
    // solves argmin || x^T H x + g^T x ||^2
    //        s.t. A x >= A_lb

    // hessian matrix
    const Eigen::Matrix<real_t, 3, 3, Eigen::RowMajor> H = Eigen::Matrix<real_t, 3, 3, Eigen::RowMajor>::Identity();
    // gradient vector
    auto direction = target_position - current_position;
    double step_size = std::min(direction.norm(), max_distance);
    Eigen::Vector3d delta_x_des = direction.normalized() * step_size;
    Eigen::Vector<real_t, 3> g = -2 * (delta_x_des);
    // constraint matrix
    Eigen::Vector<real_t, Eigen::Dynamic> A_lb(n_constraints);

    Eigen::Matrix<real_t, Eigen::Dynamic, 3, Eigen::RowMajor> A(n_constraints, 3);
    for (int i = 0; i < n_constraints; ++i)
    {
        const Eigen::Vector3d n = constraint_planes[i].first;
        const Eigen::Vector3d p = constraint_planes[i].second;
        A_lb(i) = -n.transpose() * (current_position - p) + sphere_radius;
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
#endif // VF_COMPUTATION_HPP