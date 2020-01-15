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

} // namespace hityaview