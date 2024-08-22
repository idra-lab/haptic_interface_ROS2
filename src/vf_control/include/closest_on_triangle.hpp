#ifndef TRIANGLE_CLOSEST_POINT_HPP
#define TRIANGLE_CLOSEST_POINT_HPP

#include <Eigen/Dense>
#include <cmath>
#include "location.hpp"

double round_up(double value, int decimal_places)
{
    const double multiplier = std::pow(10.0, decimal_places);
    return std::ceil(value * multiplier) / multiplier;
}
// Function to calculate the edge equation
inline double E(const Eigen::Vector2d &xy, const Eigen::Vector2d &XY, const Eigen::Vector2d &dxdy)
{
    return round_up((xy[0] - XY[0]) * dxdy[1] - (xy[1] - XY[1]) * dxdy[0], 6);
}

// Function to find the closest point on the triangle
std::pair<Eigen::Vector3d, Location> findClosestPointOnTriangle(
    const Eigen::Vector3d &point,
    const Eigen::Matrix4d &triXfm,
    const Eigen::Matrix4d &triXfmInv,
    const Eigen::Vector3d &v1,
    const Eigen::Vector3d &v2,
    const Eigen::Vector3d &v3)
{

    // Transform point to local triangle coordinates
    Eigen::Vector4d point_homogeneous(point[0], point[1], point[2], 1.0);
    Eigen::Vector4d pt_local_homogeneous = triXfm * point_homogeneous;
    Eigen::Vector2d pt_2d(pt_local_homogeneous[0], pt_local_homogeneous[1]);

    // Transform vertices to local 2D coordinates
    Eigen::Vector4d v1_homogeneous(v1[0], v1[1], v1[2], 1.0);
    Eigen::Vector4d v2_homogeneous(v2[0], v2[1], v2[2], 1.0);
    Eigen::Vector4d v3_homogeneous(v3[0], v3[1], v3[2], 1.0);

    Eigen::Vector2d P1(0.0, 0.0);
    Eigen::Vector2d P2 = (triXfm * v2_homogeneous).head<2>();
    Eigen::Vector2d P3 = (triXfm * v3_homogeneous).head<2>();

    Eigen::Vector2d P1P3 = P3 - P1;
    Eigen::Vector2d P2P3 = P3 - P2;

    Eigen::Vector2d E12(-1.0, 0.0);
    Eigen::Vector2d E13(P3[1], -P3[0]);
    Eigen::Vector2d E23(-P2P3[1], P2P3[0]);

    double norm_E13 = E13.norm();
    double norm_E23 = E23.norm();
    if (norm_E13 != 0)
    {
        E13 /= norm_E13;
    }
    if (norm_E23 != 0)
    {
        E23 /= norm_E23;
    }

    // Check which edge or vertex the point is closest to
    if (pt_2d[0] <= 0.0)
    { // Outside edge P1P2
        if (pt_2d[1] <= 0.0)
        {
            if (E(pt_2d, P1, E13) >= 0.0)
            {
                Eigen::Vector4d closest_local(P1[0], P1[1], 0.0, 1.0);
                Eigen::Vector3d closestPoint = (triXfmInv * closest_local).head<3>();
                return {closestPoint, Location::V1};
            }
        }
        else if (pt_2d[1] >= P2[1])
        {
            if (E(pt_2d, P2, E23) <= 0.0)
            {
                Eigen::Vector4d closest_local(P2[0], P2[1], 0.0, 1.0);
                Eigen::Vector3d closestPoint = (triXfmInv * closest_local).head<3>();
                return {closestPoint, Location::V2};
            }
        }
        else
        {
            Eigen::Vector4d closest_local(0.0, pt_2d[1], 0.0, 1.0);
            Eigen::Vector3d closestPoint = (triXfmInv * closest_local).head<3>();
            return {closestPoint, Location::V1V2};
        }
    }

    if (E(pt_2d, P1, P1P3) >= 0.0)
    {
        if (E(pt_2d, P1, E13) >= 0.0)
        {
            Eigen::Vector4d closest_local(P1[0], P1[1], 0.0, 1.0);
            Eigen::Vector3d closestPoint = (triXfmInv * closest_local).head<3>();
            return {closestPoint, Location::V1};
        }
        else if (E(pt_2d, P3, E13) <= 0.0)
        {
            if (E(pt_2d, P3, E23) >= 0.0)
            {
                Eigen::Vector4d closest_local(P3[0], P3[1], 0.0, 1.0);
                Eigen::Vector3d closestPoint = (triXfmInv * closest_local).head<3>();
                return {closestPoint, Location::V3};
            }
        }
        else
        {
            Eigen::Vector2d projection = pt_2d.dot(P1P3) * P1P3;
            Eigen::Vector4d closest_local(projection[0], projection[1], 0.0, 1.0);
            Eigen::Vector3d closestPoint = (triXfmInv * closest_local).head<3>();
            return {closestPoint, Location::V1V3};
        }
    }

    if (E(pt_2d, P2, P2P3) <= 0)
    {
        if (E(pt_2d, P2, E23) <= 0)
        {
            Eigen::Vector4d closest_local(P2[0], P2[1], 0.0, 1.0);
            Eigen::Vector3d closestPoint = (triXfmInv * closest_local).head<3>();
            return {closestPoint, Location::V2};
        }
        else if (E(pt_2d, P3, E23) >= 0)
        {
            Eigen::Vector4d closest_local(P3[0], P3[1], 0.0, 1.0);
            Eigen::Vector3d closestPoint = (triXfmInv * closest_local).head<3>();
            return {closestPoint, Location::V3};
        }
        else
        {
            Eigen::Vector2d projection = (pt_2d - P2).dot(P2P3) / P2P3.dot(P2P3) * P2P3 + P2;
            Eigen::Vector4d closest_local(projection[0], projection[1], 0.0, 1.0);
            Eigen::Vector3d closestPoint = (triXfmInv * closest_local).head<3>();
            return {closestPoint, Location::V2V3};
        }
    }

    Eigen::Vector4d closest_local(pt_2d[0], pt_2d[1], 0.0, 1.0);
    Eigen::Vector3d closestPoint = (triXfmInv * closest_local).head<3>();
    return {closestPoint, Location::IN};
}

Eigen::Matrix4d compute_triangle_xfm(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3)
{
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
#endif // TRIANGLE_CLOSEST_POINT_HPP