#pragma once

#include <Eigen/Dense>

namespace hityavie {

class GeometryUtility {
public:
    static Eigen::Quaterniond AngleAxis2Quat(const Eigen::Vector3d &r);
    static Eigen::Matrix3d Skew(const Eigen::Vector3d &v);
    static Eigen::Vector3d Triangulate(const Eigen::Matrix4d &tc1w, const Eigen::Vector3d &ray1, 
        const Eigen::Matrix4d &tc2w, const Eigen::Vector3d &ray2);
    static Eigen::Vector3d Transform(const Eigen::Matrix4d &tcw, const Eigen::Vector3d &pw);
    GeometryUtility() = delete;
};

} // namespace hityaview