#include "mesh.hpp"

#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>

#include "location.hpp"
#include "open3d/Open3D.h"

void testFindNearest() {
  auto o3d_mesh = open3d::geometry::TriangleMesh::CreateSphere(0.2, 40);
  Mesh mesh(o3d_mesh->vertices_, o3d_mesh->triangles_,
            o3d_mesh->vertex_normals_);
  Eigen::Vector3d point(-0.09, -0.0024, 0.20140033);
  auto nearest = mesh.find_nearby_triangles(point, 0.03);
  std::cout << "Nearest triangles: " << nearest.size() << std::endl;
  for (auto idx : nearest) {
    std::cout << idx << std::endl;
  }
}
void testClosestOnTriangle() {
  // Test Case 1: Point at vertex P1
  Eigen::Vector3d point(0.0, 0.0, 0.0);
  Eigen::Vector3d P1(0.0, 0.0, 0.0);
  Eigen::Vector3d P2(1.0, 0.0, 0.0);
  Eigen::Vector3d P3(0.0, 1.0, 0.0);
  Eigen::Matrix4d triXfm = compute_triangle_xfm(P1, P2, P3);
  Eigen::Matrix4d triXfmInv = triXfm.inverse();
  std::cout << "Transf matrix:\n"
            << triXfm << std::endl
            << "Inv:\n"
            << triXfmInv << std::endl;

  std::pair<Eigen::Vector3d, Location> closestPoint =
      findClosestPointOnTriangle(point, triXfm, triXfmInv, P1, P2, P3);
  std::cout << "Test 1" << std::endl
            << closestPoint.first
            << "\n Loc: " << LocationToString(closestPoint.second) << std::endl;
  std::cout << "-----------------------------------" << std::endl;

  // Test Case 2: Point at vertex P2
  point << 1.0, 0.0, 0.0;
  closestPoint =
      findClosestPointOnTriangle(point, triXfm, triXfmInv, P1, P2, P3);
  std::cout << "Test 2" << std::endl
            << closestPoint.first
            << "\n Loc: " << LocationToString(closestPoint.second) << std::endl;
  std::cout << "-----------------------------------" << std::endl;

  // Test Case 3: Point at vertex P3
  point << 0.0, 1.0, 0.0;
  closestPoint =
      findClosestPointOnTriangle(point, triXfm, triXfmInv, P1, P2, P3);
  std::cout << "Test 3" << std::endl
            << closestPoint.first
            << "\n Loc: " << LocationToString(closestPoint.second) << std::endl;
  std::cout << "-----------------------------------" << std::endl;

  // Test Case 4: Point inside triangle
  point << 0.5, 0.0, 0.0;
  closestPoint =
      findClosestPointOnTriangle(point, triXfm, triXfmInv, P1, P2, P3);
  std::cout << "Test 4" << std::endl
            << closestPoint.first
            << "\n Loc: " << LocationToString(closestPoint.second) << std::endl;
  std::cout << "-----------------------------------" << std::endl;

  // Test Case 5: Point on edge P1P3 but not at vertices
  point << 0.0, 0.5, 0.0;
  closestPoint =
      findClosestPointOnTriangle(point, triXfm, triXfmInv, P1, P2, P3);
  std::cout << "Test 5" << std::endl
            << closestPoint.first
            << "\n Loc: " << LocationToString(closestPoint.second) << std::endl;
  std::cout << "-----------------------------------" << std::endl;

  // Test Case 6: Point on edge P1P3 but not at vertices
  point << 0.5, 0.5, 0.0;
  closestPoint =
      findClosestPointOnTriangle(point, triXfm, triXfmInv, P1, P2, P3);
  std::cout << "Test 6" << std::endl
            << closestPoint.first
            << "\n Loc: " << LocationToString(closestPoint.second) << std::endl;
  std::cout << "-----------------------------------" << std::endl;

  // Test Case 7: Point inside triangle
  point << 0.25, 0.25, 0.0;
  closestPoint =
      findClosestPointOnTriangle(point, triXfm, triXfmInv, P1, P2, P3);
  std::cout << "Test 7" << std::endl
            << closestPoint.first
            << "\n Loc: " << LocationToString(closestPoint.second) << std::endl;
  std::cout << "-----------------------------------" << std::endl;

  // Test Case 7: Point inside triangle
  point << 0.4, 1.1, 0.0;
  closestPoint =
      findClosestPointOnTriangle(point, triXfm, triXfmInv, P1, P2, P3);
  std::cout << "Test 8" << std::endl
            << closestPoint.first
            << "\n Loc: " << LocationToString(closestPoint.second) << std::endl;
  std::cout << "-----------------------------------" << std::endl;

  point << -0.5, -1.5, 0.0;
  closestPoint =
      findClosestPointOnTriangle(point, triXfm, triXfmInv, P1, P2, P3);
  std::cout << "Test 9" << std::endl
            << closestPoint.first
            << "\n Loc: " << LocationToString(closestPoint.second) << std::endl;
  std::cout << "-----------------------------------" << std::endl;

  // Try with mesh
  auto o3d_mesh = open3d::geometry::TriangleMesh::CreateSphere(0.2, 40);
  Mesh mesh(o3d_mesh->vertices_, o3d_mesh->triangles_,
            o3d_mesh->vertex_normals_);
  point << -0.09, -0.0024, 0.20140033;
  int face = 880;
  auto xfm = mesh.triangle_xfm[face];
  auto xfm_inv = mesh.triangle_xfm_inv[face];
  std::cout << "Test 10 with face 880" << std::endl;
  std::cout << "Xfm: \n" << xfm << std::endl;
  std::cout << "Xfm_inv: \n" << xfm_inv << std::endl;
  const Eigen::Vector3d &V1 = mesh.vertices[mesh.faces[face][0]];
  const Eigen::Vector3d &V2 = mesh.vertices[mesh.faces[face][1]];
  const Eigen::Vector3d &V3 = mesh.vertices[mesh.faces[face][2]];
  std::cout << "Face 880 vertices: " << V1 << " " << V2 << " " << V3
            << std::endl;

  auto [CPi, loc] = findClosestPointOnTriangle(point, xfm, xfm_inv, V1, V2, V3);
  std::cout << "CPI: \n"
            << CPi << "\n Loc: " << LocationToString(loc) << std::endl;
}
void colorMeshFacesNew(open3d::t::geometry::TriangleMesh &mesh_new, int faceIdx,
                       std::vector<int> adjacent_faces) {
  // ComputeVertexNormals
  // mesh_new.ComputeTriangleNormals();
  auto faces = mesh_new.GetTriangleIndices();
  // Set the colors to the mesh
  auto colors = open3d::core::Tensor::Ones({faces.GetShape()[0], 3},
                                           open3d::core::Dtype::Float32);

  open3d::core::SizeVector faces_size = faces.GetShape();
  std::cout << "Faces size " << faces_size.ToString() << std::endl;
  open3d::core::SizeVector colors_size = colors.GetShape();
  std::cout << "Colors size " << colors_size.ToString() << std::endl;
  // Set the colors to the mesh
  colors[faceIdx][0] = 1;
  colors[faceIdx][1] = 0;
  colors[faceIdx][2] = 0;

  for (int i = 0; i < adjacent_faces.size(); i++) {
    colors[adjacent_faces[i]][0] = 0;
    colors[adjacent_faces[i]][1] = 1;
    colors[adjacent_faces[i]][2] = 0;
  }
  mesh_new.SetTriangleColors(colors);
}
void colorVertices(open3d::geometry::TriangleMesh &mesh, int faceIdx,
                   std::vector<int> adjacent_faces) {
  auto faces = mesh.triangles_;
  std::vector<Eigen::Vector3d> colors(mesh.vertices_.size(),
                                      Eigen::Vector3d(1.0, 1.0, 1.0));

  int v1 = faces[faceIdx][0];
  int v2 = faces[faceIdx][1];
  int v3 = faces[faceIdx][2];
  colors[v1][0] = 1;
  colors[v2][1] = 0;
  colors[v3][2] = 0;

  for (int i = 0; i < adjacent_faces.size(); i++) {
    v1 = faces[adjacent_faces[i]][0];
    v2 = faces[adjacent_faces[i]][1];
    v3 = faces[adjacent_faces[i]][2];
    colors[v1][0] = 0;
    colors[v2][1] = 0;
    colors[v3][2] = 1;
  }
  mesh.vertex_colors_ = colors;
}
void testAdjacencyList() {
  open3d::data::KnotMesh dataset;
  auto o3d_mesh = open3d::io::CreateMeshFromFile(dataset.GetPath());
  o3d_mesh->Scale(0.002, Eigen::Vector3d(0, 0, 0));

  Mesh mesh(o3d_mesh->vertices_, o3d_mesh->triangles_,
            o3d_mesh->vertex_normals_);
  std::cout << "Number of mesh vertices: " << o3d_mesh->vertices_.size()
            << std::endl;
  // random sample a face idx and color the neighbors
  for (int i = 0; i < 10; i++) {
    int face_idx = rand() % mesh.faces.size();
    // iter locations
    for (int loc = 1; loc <= 6; loc++) {
      {
        std::vector<std::shared_ptr<const open3d::geometry::Geometry>>
            geometries;
        auto sphere_face = open3d::geometry::TriangleMesh::CreateSphere(0.002);
        // compute the center of the face
        Eigen::Vector3d center =
            (o3d_mesh->vertices_[o3d_mesh->triangles_[face_idx][0]] +
             o3d_mesh->vertices_[o3d_mesh->triangles_[face_idx][1]] +
             o3d_mesh->vertices_[o3d_mesh->triangles_[face_idx][2]]) /
            3.0;
        sphere_face->Translate(center, false);
        sphere_face->ComputeVertexNormals();
        sphere_face->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
        geometries.push_back(sphere_face);
        std::cout << "Retrieving neighbors for face " << face_idx
                  << " at location "
                  << LocationToString(static_cast<Location>(loc)) << std::endl;
        auto neighbors = mesh.adjacency_dict.at({face_idx, intToLocation(loc)});
        std::cout << "Face " << face_idx << " has neighbors: \n";
        geometries.push_back(o3d_mesh);
        for (auto neighbor : neighbors) {
          // create sphere in the center of the face
          std::cout << neighbor << std::endl;
          auto sphere = open3d::geometry::TriangleMesh::CreateSphere(0.002);
          // compute the center of the face
          Eigen::Vector3d center =
              (o3d_mesh->vertices_[o3d_mesh->triangles_[neighbor][0]] +
               o3d_mesh->vertices_[o3d_mesh->triangles_[neighbor][1]] +
               o3d_mesh->vertices_[o3d_mesh->triangles_[neighbor][2]]) /
              3.0;
          sphere->Translate(center, false);
          sphere->ComputeVertexNormals();
          sphere->PaintUniformColor(Eigen::Vector3d(0, 1, 0));
          geometries.push_back(sphere);
        }
        open3d::visualization::DrawGeometries(geometries);
      }
    }
  }
}
void testExtrusion(){
  auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
  open3d::io::ReadTriangleMesh("/home/nardi/SKEL_WS/ros2_ws/projected_skel.obj", *o3d_mesh);
  open3d::visualization::DrawGeometries({o3d_mesh});
  o3d_mesh->ComputeTriangleNormals();
  Mesh mesh(o3d_mesh->vertices_, o3d_mesh->triangles_,
            o3d_mesh->vertex_normals_);
  mesh.extrudeMeshRadially(Eigen::Vector3d(0, 0, 0), 0.05);
  auto o3d_mesh_extruded = std::make_shared<open3d::geometry::TriangleMesh>();
  o3d_mesh_extruded->vertices_ = mesh.vertices;
  o3d_mesh_extruded->triangles_ = mesh.faces;
  o3d_mesh_extruded->ComputeTriangleNormals();
  open3d::visualization::DrawGeometries({o3d_mesh_extruded});


}
int main(int argc, char **argv) {
  // testFindNearest();
  // exit(0);
  // testClosestOnTriangle();
  // testAdjacencyList();
  testExtrusion();
}