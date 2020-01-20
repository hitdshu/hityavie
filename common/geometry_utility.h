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
    static Eigen::Matrix4d QuatLmat(const Eigen::Vector4d &quat);
    static Eigen::Matrix4d QuatLmat(const Eigen::Quaterniond &quat);
    static Eigen::Matrix4d QuatRmat(const Eigen::Vector4d &quat);
    static Eigen::Matrix4d QuatRmat(const Eigen::Quaterniond &quat);
    static Eigen::Matrix<double, 7, 1> Pose2Vec(const Eigen::Matrix4d &pose);
    static Eigen::Matrix4d Vec2Pose(const Eigen::Matrix<double, 7, 1> &vec);
    GeometryUtility() = delete;
};

} // namespace hityaview