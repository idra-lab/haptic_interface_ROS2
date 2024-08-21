#ifndef MESH_HPP
#define MESH_HPP
#include "open3d/Open3D.h"
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <set>
#include <tuple>
#include <iostream>
#include <fstream>
#include <omp.h>
#include <cmath>
#include "location.hpp"
// nlohmann json
#include "json.hpp"

struct hash_tuple
{
    template <class T1, class T2>
    std::size_t operator()(const std::tuple<T1, T2> &t) const
    {
        auto h1 = std::hash<T1>{}(std::get<0>(t));
        auto h2 = std::hash<T2>{}(std::get<1>(t));
        return h1 ^ h2;
    }
};
class Mesh
{
public:
    Mesh(const std::vector<Eigen::Vector3d> &vertices,
         const std::vector<Eigen::Vector3i> &faces,
         const std::vector<Eigen::Vector3d> &normals)
        : vertices(vertices), faces(faces), normals(normals)
    {
        std::cout << "Creating KDTree" << std::endl;
        // Build a KDTree for the point cloud
        // Convert vertices to a PointCloud
        point_cloud_ = std::make_shared<open3d::geometry::PointCloud>();
        point_cloud_->points_ = vertices;

        kdtree.SetGeometry(*point_cloud_);
        std::cout << "Precomputing transformation matrices" << std::endl;
        precompute_triangle_xfm();
        std::cout << "Precomputing adjacency dictionary" << std::endl;
        precompute_adjacency();
    }
    std::vector<int> find_nearby_triangles(const Eigen::Vector3d &position, double max_distance)
    {
        std::vector<int> indices;
        std::vector<double> distances;
        kdtree.SearchRadius(position, max_distance, indices, distances);

        std::unordered_set<int> index_set(indices.begin(), indices.end());
        std::vector<int> nearby_triangles;

        for (size_t i = 0; i < faces.size(); ++i)
        {
            auto face = faces[i];
            for (auto vertex_index : face)
            {
                if (index_set.find(vertex_index) != index_set.end())
                {
                    nearby_triangles.push_back(i);
                    break; // No need to check other vertices of this face
                }
            }
        }

        return nearby_triangles;
    }
    std::pair<Eigen::Vector3d, Location> get_closest_on_triangle(const Eigen::Vector3d &point, int face)
    {
        const Eigen::Vector3d &V1 = vertices[faces[face][0]];
        const Eigen::Vector3d &V2 = vertices[faces[face][1]];
        const Eigen::Vector3d &V3 = vertices[faces[face][2]];
        return findClosestPointOnTriangle(point, triangle_xfm[face], triangle_xfm_inv[face], V1, V2, V3);
    }
    void precompute_triangle_xfm()
    {
        triangle_xfm.resize(faces.size());
        triangle_xfm_inv.resize(faces.size());
        for (size_t i = 0; i < faces.size(); ++i)
        {
            std::cout << i << "/" << faces.size() << std::endl;
            triangle_xfm[i] = compute_triangle_xfm(i);
            triangle_xfm_inv[i] = triangle_xfm[i].inverse();
        }
    }

    Eigen::Matrix4d compute_triangle_xfm(int idx)
    {
        Eigen::Vector3d v1 = vertices[faces[idx][0]];
        Eigen::Vector3d v2 = vertices[faces[idx][1]];
        Eigen::Vector3d v3 = vertices[faces[idx][2]];

        Eigen::Vector3d yaxis = (v2 - v1).normalized();
        Eigen::Vector3d zaxis = (v3 - v1).cross(yaxis).normalized();
        Eigen::Vector3d xaxis = yaxis.cross(zaxis).normalized();

        Eigen::Matrix3d R;
        R << xaxis, yaxis, zaxis;

        Eigen::Vector3d T = -R * v1;

        Eigen::Matrix4d xfm = Eigen::Matrix4d::Identity();
        xfm.block<3, 3>(0, 0) = R;
        xfm.block<3, 1>(0, 3) = T;

        return xfm;
    }

    void precompute_adjacency()
    {
        if (load_adjacency_dict("adjacency_dict.txt"))
        {
            return;
        }

        std::unordered_map<std::tuple<int, Location>, std::vector<int>, hash_tuple> adjacency_dict;

#pragma omp parallel for
        for (size_t i = 0; i < faces.size(); ++i)
        {
            std::cout << i << "/" << faces.size() << std::endl;
            for (Location location : {Location::V1, Location::V2, Location::V3,
                                      Location::V1V2, Location::V1V3, Location::V2V3})
            {
                std::vector<int> adjacent_faces = get_adjacent_faces(i, location);
#pragma omp critical
                {
                    adjacency_dict[{i, location}] = adjacent_faces;
                }
            }
        }

        save_adjacency_dict("adjacency_dict.txt", adjacency_dict);
        this->adjacency_dict = adjacency_dict;
    }

    std::vector<int> get_adjacent_faces(int face_index, Location location)
    {
        std::vector<int> vertices_to_query;
        const Eigen::Vector3i &face_vertices = faces[face_index];

        switch (location)
        {
        case Location::V1:
            vertices_to_query = {face_vertices[0]};
            break;
        case Location::V2:
            vertices_to_query = {face_vertices[1]};
            break;
        case Location::V3:
            vertices_to_query = {face_vertices[2]};
            break;
        case Location::V1V2:
            vertices_to_query = {face_vertices[0], face_vertices[1]};
            break;
        case Location::V1V3:
            vertices_to_query = {face_vertices[0], face_vertices[2]};
            break;
        case Location::V2V3:
            vertices_to_query = {face_vertices[1], face_vertices[2]};
            break;
        default:
            return {}; // Invalid location
        }

        std::vector<int> adjacent_faces;
        for (size_t i = 0; i < faces.size(); ++i)
        {
            const Eigen::Vector3i &test_face = faces[i];
            int matches = 0;
            for (int v : vertices_to_query)
            {
                if (test_face[0] == v || test_face[1] == v || test_face[2] == v)
                {
                    ++matches;
                }
            }
            if (matches == (int)vertices_to_query.size() && (int)i != face_index)
            {
                adjacent_faces.push_back(i);
            }
        }
        return adjacent_faces;
    }

    bool is_locally_convex(int idx, int neighbor, Location location)
    {
        return check_convexity(idx, neighbor, location);
    }

    bool is_locally_concave(int idx, int neighbor, Location location)
    {
        return !is_locally_convex(idx, neighbor, location);
    }

    bool check_convexity(int idx, int idxNeighbor, Location location)
    {
        const Eigen::Vector3i &current_triangle = faces[idx];
        Eigen::Vector3d vec1, vec2;

        switch (location)
        {
        case Location::V1V2:
            vec1 = vertices[current_triangle[2]] - vertices[current_triangle[0]];
            vec2 = vertices[current_triangle[2]] - vertices[current_triangle[1]];
            break;
        case Location::V1V3:
            vec1 = vertices[current_triangle[1]] - vertices[current_triangle[0]];
            vec2 = vertices[current_triangle[1]] - vertices[current_triangle[2]];
            break;
        case Location::V2V3:
            vec1 = vertices[current_triangle[0]] - vertices[current_triangle[1]];
            vec2 = vertices[current_triangle[0]] - vertices[current_triangle[2]];
            break;
        case Location::V1:
            vec1 = vertices[current_triangle[1]] - vertices[current_triangle[0]];
            vec2 = vertices[current_triangle[2]] - vertices[current_triangle[0]];
            break;
        case Location::V2:
            vec1 = vertices[current_triangle[0]] - vertices[current_triangle[1]];
            vec2 = vertices[current_triangle[2]] - vertices[current_triangle[1]];
            break;
        case Location::V3:
            vec1 = vertices[current_triangle[0]] - vertices[current_triangle[2]];
            vec2 = vertices[current_triangle[1]] - vertices[current_triangle[2]];
            break;
        default:
            return false; // Invalid location
        }

        const Eigen::Vector3d &neighbor_normal = normals[idxNeighbor];

        if (vec1.dot(neighbor_normal) > 0.0 || vec2.dot(neighbor_normal) > 0.0)
        {
            return true;
        }
        else if (vec1.dot(neighbor_normal) < 0.0 || vec2.dot(neighbor_normal) < 0.0)
        {
            return false;
        }
        return false;
    }

    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> faces;
    std::vector<Eigen::Vector3d> normals;
    std::unordered_map<std::tuple<int, Location>, std::vector<int>, hash_tuple> adjacency_dict;

private:
    std::vector<Eigen::Matrix4d> triangle_xfm;
    std::vector<Eigen::Matrix4d> triangle_xfm_inv;
    open3d::geometry::KDTreeFlann kdtree;
    std::shared_ptr<open3d::geometry::PointCloud> point_cloud_;

    bool load_adjacency_dict(const std::string &filename)
    {
        std::ifstream ifs(filename, std::ios::binary);
        if (!ifs.is_open())
        {
            return false;
        }
        nlohmann::json j;
        ifs >> j;
        adjacency_dict = j.get<std::unordered_map<std::tuple<int, Location>, std::vector<int>, hash_tuple>>();
        return true;
    }

    void save_adjacency_dict(const std::string &filename,
                             const std::unordered_map<std::tuple<int, Location>, std::vector<int>, hash_tuple> &dict)
    {
        // serialize with json nlohmann
        std::ofstream ofs(filename, std::ios::binary);
        if (!ofs.is_open())
        {
            return;
        }
        nlohmann::json j = dict;
        ofs << j.dump(4);
    }
};

#endif // MESH_HPP