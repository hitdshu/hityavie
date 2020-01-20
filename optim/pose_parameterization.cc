#include "optim/pose_parameterization.h"
#include "common/geometry_utility.h"

namespace hityavie {

bool PoseParameterization::Plus(const double *x, const double *delta, double *x_plus) const {
    Eigen::Map<const Eigen::Vector3d> p(x);
    Eigen::Map<const Eigen::Quaterniond> q(x + 3);
    Eigen::Map<const Eigen::Vector3d> dp(delta);
    Eigen::Quaterniond dq = GeometryUtility::AngleAxis2Quat(Eigen::Map<const Eigen::Vector3d>(delta + 3));
    Eigen::Map<Eigen::Vector3d> np(x_plus);
    Eigen::Map<Eigen::Quaterniond> nq(x_plus + 3);
    np = p + dp;
    nq = (q * dq).normalized();
    return true;
}

bool PoseParameterization::ComputeJacobian(const double *x, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(jacobian);
    J.topRows<6>().setIdentity();
    J.bottomRows<1>().setZero();
    return true;
}

} // namespace hityavie