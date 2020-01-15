#pragma once

#include <Eigen/Dense>

namespace hityavie {

class GeometryUtility {
public:
    static Eigen::Quaterniond AngleAxis2Quat(const Eigen::Vector3d &r);
    static Eigen::Matrix3d Skew(const Eigen::Vector3d &v);
    
    GeometryUtility() = delete;
};

} // namespace hityaview