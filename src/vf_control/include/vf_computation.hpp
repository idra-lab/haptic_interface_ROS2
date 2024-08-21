#ifndef VF_COMPUTATION_HPP
#define VF_COMPUTATION_HPP

#include "mesh.hpp" // Include the Mesh class and other necessary headers

Eigen::Vector3d _enforce_virtual_fixture(Mesh mesh, const Eigen::Vector3d &target_position,const Eigen::Vector3d &current_position, double sphere_radius) {
    // Define a small buffer distance to prevent penetration
    const double lookup_area = sphere_radius;

    // Find nearby triangles within a specified distance
    const double max_distance = sphere_radius + lookup_area;
    std::vector<int> nearby_triangles = mesh.find_nearby_triangles(current_position, max_distance);

    if (nearby_triangles.empty()) {
        // No nearby triangles found; return the target position
        return target_position - current_position;
    }

    // Construct constraints based on distances to nearby triangles
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> constraint_planes;

    const double eps = 0.001;
    // Eigen::VectorXd T = Eigen::Map<Eigen::VectorXd>(nearby_triangles.data(), nearby_triangles.size());
    Eigen::VectorXi T(nearby_triangles.size());
    for (size_t i = 0; i < nearby_triangles.size(); ++i) {
        T(i) = nearby_triangles[i];
    }
      

    // Precompute CPi for all triangles
    std::unordered_map<int, std::pair<Eigen::Vector3d, Location>> CP;
    for (int Ti : T) {
        CP[Ti] = mesh.get_closest_on_triangle(current_position, Ti);
    }

    int l = 0; // List index

    for (int i = 0; i < T.size(); ++i) {
        int Ti = T[i];
        Eigen::Vector3d CPi;
        Location cpi_loc;
        std::tie(CPi, cpi_loc) = CP[Ti];

        Eigen::Vector3d Ni = mesh.normals[Ti];
        Ni.normalize();

        // Check if CPi is in the triangle and the normal points towards the sphere center
        if (cpi_loc == Location::IN) {
            if (Ni.dot(current_position - CPi) >= 0) {
                constraint_planes.push_back({Ni, CPi});
                continue;
            }
        } else {
            // Handle points on the edges or vertices
            if (cpi_loc == Location::V1 || cpi_loc == Location::V2 || cpi_loc == Location::V3) {
                Eigen::Vector3d n = Ni;
                if ((current_position - CPi).norm() < eps) {
                    for (int face : T.tail(T.size() - i - 1)) {
                        Eigen::Vector3d CPia;
                        Location _;
                        std::tie(CPia, _) = CP[face];
                        if ((CPia - CPi).norm() < eps) {
                            n += mesh.normals[face].normalized();
                            T = Eigen::VectorXi::Zero(T.size() - 1);
                        }
                    }
                    n.normalize();
                    constraint_planes.push_back({n, CPi});
                    ++l;
                    continue;
                } else {
                    std::vector<int> neighborIdx1, neighborIdx2;

                    if (cpi_loc == Location::V1) {
                        neighborIdx1 = mesh.adjacency_dict[{Ti, Location::V1V2}];
                        neighborIdx2 = mesh.adjacency_dict[{Ti, Location::V1V3}];
                    } else if (cpi_loc == Location::V2) {
                        neighborIdx1 = mesh.adjacency_dict[{Ti, Location::V1V2}];
                        neighborIdx2 = mesh.adjacency_dict[{Ti, Location::V2V3}];
                    } else if (cpi_loc == Location::V3) {
                        neighborIdx1 = mesh.adjacency_dict[{Ti, Location::V1V3}];
                        neighborIdx2 = mesh.adjacency_dict[{Ti, Location::V2V3}];
                    } else {
                        std::cerr << "Error: location not on vertex" << std::endl;
                        exit(EXIT_FAILURE);
                    }

                    bool keep = false;
                    if (!neighborIdx1.empty()) {
                        int neighborIdx = neighborIdx1[0];
                        Eigen::Vector3d CPia;
                        Location _;
                        std::tie(CPia, _) = CP[neighborIdx];
                        if ((CPia - CPi).norm() < eps) {
                            T = Eigen::VectorXi::Zero(T.size() - 1);
                            keep = true;
                        }
                    }
                    if (!neighborIdx2.empty()) {
                        int neighborIdx = neighborIdx2[0];
                        Eigen::Vector3d CPia;
                        Location _;
                        std::tie(CPia, _) = CP[neighborIdx];
                        if ((CPia - CPi).norm() < eps) {
                            T = Eigen::VectorXi::Zero(T.size() - 1);
                            keep = true;
                        }
                    }
                    if (keep) {
                        n = current_position - CPi;
                        n.normalize();
                        constraint_planes.push_back({n, CPi});
                        ++l;
                        continue;
                    }
                }
            } else if (cpi_loc != Location::VOID) {
                int neighborIdx = mesh.adjacency_dict[{Ti, cpi_loc}].front();
                Eigen::Vector3d CPia;
                Location cpia_loc;
                std::tie(CPia, cpia_loc) = CP[neighborIdx];

                if (mesh.is_locally_concave(Ti, neighborIdx, cpi_loc)) {
                    if ((CPi - CPia).norm() < eps) {
                        if ((current_position - CPi).norm() < eps) {
                            Ni += mesh.normals[neighborIdx].normalized();
                            Ni.normalize();
                            constraint_planes.push_back({Ni, CPi});
                            ++l;
                            continue;
                        } else {
                            T = Eigen::VectorXi::Zero(T.size() - 1);
                            Eigen::Vector3d n = current_position - CPi;
                            n.normalize();
                            constraint_planes.push_back({n, CPi});
                            ++l;
                            continue;
                        }
                    }
                } else if (cpia_loc != Location::VOID && Ni.dot(current_position - CPi) >= 0) {
                    constraint_planes.push_back({Ni, CPi});
                    ++l;
                    continue;
                }
            }

            CP[Ti] = {CP[Ti].first, Location::VOID};
            T = Eigen::VectorXi::Zero(T.size() - 1);
        }
    }

    // Now you can process constraint_planes as needed
    // Implement logic to return the correct adjusted position
    return target_position; // Replace with the correct computation based on constraints
}
#endif // VF_COMPUTATION_HPP