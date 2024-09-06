#ifndef MESH_HPP
#define MESH_HPP
#include <omp.h>

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "open3d/Open3D.h"
#include "closest_on_triangle.hpp"
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
    point_cloud_ = std::make_shared<open3d::geometry::PointCloud>();
    point_cloud_->points_ = vertices;

    kdtree.SetGeometry(*point_cloud_);
    std::cout << "Precomputing transformation matrices" << std::endl;
    precompute_triangle_xfm();
    std::cout << "Precomputing adjacency dictionary" << std::endl;
    precompute_adjacency();
  }
  std::vector<int> find_nearby_triangles(const Eigen::Vector3d &position,
                                         double max_distance, std::vector<int> &nearby_triangles)
  {
    std::vector<double> distances;
    std::vector<int> indices;
    kdtree.SearchRadius(position, max_distance, indices, distances);

    std::unordered_set<int> index_set(indices.begin(), indices.end());

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
  std::pair<Eigen::Vector3d, Location> get_closest_on_triangle(
      const Eigen::Vector3d &point, int face)
  {
    const Eigen::Vector3d &V1 = vertices[faces[face][0]];
    const Eigen::Vector3d &V2 = vertices[faces[face][1]];
    const Eigen::Vector3d &V3 = vertices[faces[face][2]];
    return findClosestPointOnTriangle(point, triangle_xfm[face],
                                      triangle_xfm_inv[face], V1, V2, V3);
  }
  void precompute_triangle_xfm()
  {
    triangle_xfm.resize(faces.size());
    triangle_xfm_inv.resize(faces.size());
#pragma omp parallel for
    for (size_t i = 0; i < faces.size(); ++i)
    {
      std::cout << i << "/" << faces.size() << std::endl;
      triangle_xfm[i] = compute_triangle_xfm(i);
      triangle_xfm_inv[i] = triangle_xfm[i].inverse();
    }
  }
  void extrudeMeshRadially(const Eigen::Vector3d& origin, const double scale)
  {
    (void)origin;
    // find the faces with no neighbors
    std::vector<int> border_faces;

    // #pragma omp parallel for
    const int num_faces = faces.size();
    for (size_t i = 0; (int)i < num_faces; ++i)
    {
      // const Eigen::Vector3d face_center = (vertices[faces[i][0]] + vertices[faces[i][1]] + vertices[faces[i][2]]) / 3;
      // Eigen::Vector3d direction = (origin - face_center).normalized();
      const Eigen::Vector3d direction(0,0,-1);

      if (adjacency_dict.at({i, Location::V1V2}).size() == 0)
      {
        std::cout << "Extruding face " << i << " along V1V2" << std::endl;
        extrudeEdgeAlongDirection(faces[i][0], faces[i][1], direction, scale);
      }
      if (adjacency_dict.at({i, Location::V2V3}).size() == 0)
      {
        std::cout << "Extruding face " << i << " along V2V3" << std::endl;
        extrudeEdgeAlongDirection(faces[i][1], faces[i][2], direction, scale);
      }
      if (adjacency_dict.at({i, Location::V1V3}).size() == 0)
      {
        std::cout << "Extruding face " << i << " along V1V3" << std::endl;
        extrudeEdgeAlongDirection(faces[i][0], faces[i][2], direction, scale);
      }
    }
    std::cout << "Face before extrusion: " << num_faces << std::endl;
    std::cout << "Face after extrusion: " << faces.size() << std::endl;

  }
  void extrudeEdgeAlongDirection(const int v1,const int v2, const Eigen::Vector3d &direction, const double scale)
  {
    // extrude the border faces
    const Eigen::Vector3d v1_ = vertices[v1];
    const Eigen::Vector3d v2_ = vertices[v2];
    const Eigen::Vector3d edge_centre = (v1_ + v2_) / 2;
    const Eigen::Vector3d extruded_v3 = edge_centre + direction * scale;
    // add the new vertex
    vertices.push_back(extruded_v3);
    // add the new face
    faces.push_back({v1, v2, (int)vertices.size() - 1});
  }

  Eigen::Matrix4d compute_triangle_xfm(int idx)
  {
    const Eigen::Vector3d v1 = vertices[faces[idx][0]];
    const Eigen::Vector3d v2 = vertices[faces[idx][1]];
    const Eigen::Vector3d v3 = vertices[faces[idx][2]];

    const Eigen::Vector3d yaxis = (v2 - v1).normalized();
    const Eigen::Vector3d zaxis = ((v3 - v1).normalized()).cross(yaxis).normalized();
    const Eigen::Vector3d xaxis = yaxis.cross(zaxis).normalized();

    Eigen::Matrix3d R;
    // stack
    R.block<1, 3>(0, 0) = xaxis.transpose();
    R.block<1, 3>(1, 0) = yaxis.transpose();
    R.block<1, 3>(2, 0) = zaxis.transpose();
    // -R @ v1
    Eigen::Vector3d T = -R * v1;

    Eigen::Matrix4d xfm = Eigen::Matrix4d::Identity();
    xfm.block<3, 3>(0, 0) = R;
    xfm.block<3, 1>(0, 3) = T;

    return xfm;
  }

  void precompute_adjacency()
  {
    if (load_adjacency_dict("adjacency_dict" + std::to_string(faces.size()) +
                            ".txt"))
    {
      std::cout << "Loaded adjacency dict from file" << std::endl;
      return;
    }

    std::unordered_map<std::tuple<int, Location>, std::vector<int>, hash_tuple>
        adjacency_dict;
#pragma omp parallel for
    for (size_t i = 0; i < faces.size(); ++i)
    {
      std::cout << i << "/" << faces.size() << std::endl;
      for (Location location :
           {Location::V1, Location::V2, Location::V3, Location::V1V2,
            Location::V1V3, Location::V2V3})
      {
        std::vector<int> adjacent_faces = get_adjacent_faces(i, location);
#pragma omp critical
        {
          adjacency_dict[{i, location}] = adjacent_faces;
        }
      }
    }

    save_adjacency_dict("adjacency_dict" + std::to_string(faces.size()) +
                            ".txt",
                        adjacency_dict);
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
    else if (vec1.dot(neighbor_normal) < 0.0 ||
             vec2.dot(neighbor_normal) < 0.0)
    {
      return false;
    }
    return false;
  }

  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> faces;
  std::vector<Eigen::Vector3d> normals;
  std::unordered_map<std::tuple<int, Location>, std::vector<int>, hash_tuple>
      adjacency_dict;
  std::vector<Eigen::Matrix4d> triangle_xfm;
  std::vector<Eigen::Matrix4d> triangle_xfm_inv;

private:
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
    adjacency_dict = j.get<std::unordered_map<std::tuple<int, Location>,
                                              std::vector<int>, hash_tuple>>();
    return true;
  }

  void save_adjacency_dict(
      const std::string &filename,
      const std::unordered_map<std::tuple<int, Location>, std::vector<int>,
                               hash_tuple> &dict)
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