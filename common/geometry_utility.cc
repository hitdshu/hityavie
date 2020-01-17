#include "common/geometry_utility.h"

namespace hityavie {

Eigen::Quaterniond GeometryUtility::AngleAxis2Quat(const Eigen::Vector3d &r) {
    Eigen::Quaterniond quat;
    if (r.norm() > 1e-5) {
        double theta = r.norm();
        Eigen::Vector3d axis = r / r.norm();
        quat.w() = cosf(theta / 2);
        quat.x() = axis[0] * sinf(theta / 2); 
        quat.y() = axis[1] * sinf(theta / 2); 
        quat.z() = axis[2] * sinf(theta / 2); 
    } else {
        quat = Eigen::Quaterniond(1, r[0] / 2, r[1] / 2, r[2] / 2);
        quat.normalize();
    }
    return quat;
}

Eigen::Matrix3d GeometryUtility::Skew(const Eigen::Vector3d &v) {
    Eigen::Matrix3d mat;
    mat << 0, -v[2], v[1],
        v[2], 0, -v[0],
        -v[1], v[0], 0;
    return mat;
}

Eigen::Vector3d GeometryUtility::Triangulate(const Eigen::Matrix4d &tc1w, const Eigen::Vector3d &ray1, 
    const Eigen::Matrix4d &tc2w, const Eigen::Vector3d &ray2) {
    Eigen::Vector3d ray1n = ray1 / ray1[2];
    Eigen::Vector3d ray2n = ray2 / ray2[2];
    Eigen::Matrix4d A;
    A.row(0) = tc1w.row(0) - ray1n[0] * tc1w.row(2);
    A.row(1) = tc1w.row(1) - ray1n[1] * tc1w.row(2);
    A.row(2) = tc2w.row(0) - ray2n[0] * tc2w.row(2);
    A.row(3) = tc2w.row(1) - ray2n[1] * tc2w.row(2);
    Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector4d v_last = svd.matrixV().col(3);
    Eigen::Vector3d result;
    result = v_last.block(0,0,3,1) / v_last[3];
    return result;
}

Eigen::Vector3d GeometryUtility::Transform(const Eigen::Matrix4d &tcw, const Eigen::Vector3d &pw) {
    Eigen::Vector4d pwh;
    pwh << pw[0], pw[1], pw[2], 1;
    Eigen::Vector4d pch = tcw * pwh;
    return pch.block(0, 0, 3, 1) / pch[3];
}

Eigen::Matrix4d GeometryUtility::QuatLmat(const Eigen::Vector4d &quat) {
    Eigen::Matrix4d ql = quat[0] * Eigen::Matrix4d::Identity();
    ql.block(1, 0, 3, 1) += quat.block(1, 0, 3, 1);
    ql.block(0, 1, 1, 3) -= quat.block(1, 0, 3, 1).transpose();
    ql.block(1, 1, 3, 3) += Skew(quat.block(1, 0, 3, 1));
    return ql;
}

Eigen::Matrix4d GeometryUtility::QuatLmat(const Eigen::Quaterniond &quat) {
    Eigen::Vector4d quatv;
    quatv << quat.w(), quat.x(), quat.y(), quat.z();
    return QuatLmat(quatv);
}

Eigen::Matrix4d GeometryUtility::QuatRmat(const Eigen::Vector4d &quat) {
    Eigen::Matrix4d ql = quat[0] * Eigen::Matrix4d::Identity();
    ql.block(1, 0, 3, 1) += quat.block(1, 0, 3, 1);
    ql.block(0, 1, 1, 3) -= quat.block(1, 0, 3, 1).transpose();
    ql.block(1, 1, 3, 3) -= Skew(quat.block(1, 0, 3, 1));
    return ql;
}

Eigen::Matrix4d GeometryUtility::QuatRmat(const Eigen::Quaterniond &quat) {
    Eigen::Vector4d quatv;
    quatv << quat.w(), quat.x(), quat.y(), quat.z();
    return QuatRmat(quatv);
}

} // namespace hityaview