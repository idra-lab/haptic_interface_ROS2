#ifndef VF_COMPUTATION_HPP
#define VF_COMPUTATION_HPP

#include "mesh.hpp"
#include "qp_wrapper.hpp"

Eigen::Vector3d enforce_virtual_fixture(Mesh &mesh, const Eigen::Vector3d &target_position,
                                        const Eigen::Vector3d &current_position, const double sphere_radius)
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
    // Construct constraints based on distances to nearby triangles
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> constraint_planes;

    const double eps = 0.001;
    // Eigen::VectorXd T = Eigen::Map<Eigen::VectorXd>(nearby_triangles.data(), nearby_triangles.size());
    Eigen::VectorXi T(nearby_triangles.size());
    for (size_t i = 0; i < nearby_triangles.size(); ++i)
    {
        T(i) = nearby_triangles[i];
    }
    // Precompute CPi for all triangles
    std::unordered_map<int, std::pair<Eigen::Vector3d, Location>> CP;
    int i = 0;
    for (int Ti : T)
    {
        CP[Ti] = mesh.get_closest_on_triangle(current_position, Ti);
        i++;
    }

    // Process constraints
    for (int Ti : T)
    {
        auto [CPi, cpi_loc] = CP[Ti];
        Eigen::Vector3d Ni = mesh.normals[Ti];
        Ni.normalize();

        if (cpi_loc == Location::IN)
        {
            if (Ni.dot(current_position - CPi) >= 0)
            {
                constraint_planes.emplace_back(Ni, CPi);
                continue;
            }
        }
        else if (cpi_loc == Location::V1 || cpi_loc == Location::V2 || cpi_loc == Location::V3)
        {
            Eigen::Vector3d n = Ni;
            if ((current_position - CPi).norm() < eps)
            {
                for (auto &face : T)
                {
                    auto [CPia, _] = CP[face];
                    if ((CPia - CPi).norm() < eps)
                    {
                        n += mesh.normals[face].normalized();
                        // Remove face from T
                        T = (T.array() != face).select(T, Eigen::VectorXi::Zero(T.size()));
                        CP[face].second = Location::VOID;
                    }
                }
                n.normalize();
                constraint_planes.emplace_back(n, CPi);
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
                else
                {
                    std::cerr << "Error: location not on vertex" << std::endl;
                    exit(EXIT_FAILURE);
                }

                bool keep = false;
                for (auto neighborIdx : {neighborIdx1, neighborIdx2})
                {
                    if (!neighborIdx.empty())
                    {
                        int neighbor = neighborIdx[0];
                        auto [CPia, _] = CP[neighbor];
                        if (CPia == Eigen::Vector3d::Zero())
                        {
                            CPia = mesh.get_closest_on_triangle(current_position, neighbor).first;
                        }
                        if ((CPia - CPi).norm() < eps)
                        {
                            T = (T.array() != neighbor).select(T, Eigen::VectorXi::Zero(T.size()));
                            CP[neighbor].second = Location::VOID;
                            keep = true;
                        }
                    }
                }
                if (keep)
                {
                    Eigen::Vector3d n = (current_position - CPi).normalized();
                    constraint_planes.emplace_back(n, CPi);
                    continue;
                }
            }
        }
        else if (cpi_loc != Location::VOID)
        {
            std::vector<int> neighborIdx = mesh.adjacency_dict.at({Ti, cpi_loc});
            if (!neighborIdx.empty())
            {
                int neighborIdx0 = neighborIdx[0];
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
                            continue;
                        }
                        else
                        {
                            CP[neighborIdx0].second = Location::VOID;
                            T = (T.array() != neighborIdx0).select(T, Eigen::VectorXi::Zero(T.size()));
                            Eigen::Vector3d n = (current_position - CPi).normalized();
                            constraint_planes.emplace_back(n, CPi);
                            continue;
                        }
                    }
                }
                else if (cpia_loc != Location::VOID && Ni.dot(current_position - CPi) >= 0)
                {
                    constraint_planes.emplace_back(Ni, CPi);
                    continue;
                }
            }
        }

        // Otherwise, remove current triangle
        CP[Ti].second = Location::VOID;
        T = (T.array() != Ti).select(T, Eigen::VectorXi::Zero(T.size()));
    }

    // reset constraint_planes
    // constraint_planes.clear();
    int n_constraints = constraint_planes.size();
    // std::cout << "Number of constraints: " << n_constraints << std::endl;

    // hessian matrix
    Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::RowMajor> H(3, 3);
    H.setIdentity();

    // gradient vector
    auto direction = target_position - current_position;
    double step_size = std::min(direction.norm(), max_distance);
    Eigen::Vector3d delta_x_des = direction.normalized() * step_size;
    Eigen::Vector<real_t, 3> g = - 2 *(delta_x_des);
    // constraint matrix
    Eigen::Vector<real_t, Eigen::Dynamic> A_lb(n_constraints);
    Eigen::Vector<real_t, 3> x_lb, x_ub;
    
    x_lb.setConstant(-std::numeric_limits<real_t>::infinity());
    x_ub.setConstant(std::numeric_limits<real_t>::infinity());

    Eigen::Matrix<real_t, Eigen::Dynamic, 3, Eigen::RowMajor> A(n_constraints, 3);
    for (int i = 0; i < n_constraints; ++i)
    {
        Eigen::Vector3d n = constraint_planes[i].first;
        Eigen::Vector3d p = constraint_planes[i].second;
        A_lb(i) = -n.transpose() * (current_position - p);
        A(i, 0) = n[0];
        A(i, 1) = n[1];
        A(i, 2) = n[2];
    
    }
    qpOASES::Options myOptions;
    myOptions.printLevel = qpOASES::PL_LOW;
    qpOASES::QProblem min_problem(3, n_constraints);
    min_problem.setOptions( myOptions );
    Eigen::Vector<real_t, 3> delta_x;
    int nWSR = 10;
    min_problem.init(H.data(), g.data(), A.data(), x_lb.data(), x_ub.data(), A_lb.data(), 0, nWSR);
    min_problem.getPrimalSolution(delta_x.data());
    std::cout << "delta_x: " << delta_x << std::endl;
    return delta_x.cast<double>();
}
#endif // VF_COMPUTATION_HPP