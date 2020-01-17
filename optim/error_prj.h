#pragma once

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace hityavie {

class ErrorPrj {
public:
    ErrorPrj(const Eigen::Vector2d &p_obs, const Eigen::Vector2d &sqrt_information, const Eigen::Matrix3d &k) {
        p_obs_ = p_obs;
        sqrt_information_ = sqrt_information;
        k_ = k;
    }

    template <typename T>
    bool operator()(const T* const r_cw, const T* const t_cw, const T* const p_w, T* residuals) const {
        T fx(k_(0, 0));
        T fy(k_(1, 1));
        T cx(k_(0, 2));
        T cy(k_(1, 2));
        T p[3];
        ceres::QuaternionRotatePoint(r_cw, p_w, p);
        p[0] += t_cw[0];
        p[1] += t_cw[1];
        p[2] += t_cw[2];
        T xp = p[0] / p[2] * fx + cx;
        T yp = p[1] / p[2] * fy + cy; 
        residuals[0] = (xp - T(p_obs_[0])) * sqrt_information_[0];
        residuals[1] = (yp - T(p_obs_[1])) * sqrt_information_[1];
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d &p_obs, const Eigen::Vector2d &sqrt_information, const Eigen::Matrix3d &k) {
        return (new ceres::AutoDiffCostFunction<ErrorPrj, 2, 4, 3, 3>(new ErrorPrj(p_obs, sqrt_information, k)));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    Eigen::Vector2d p_obs_;
    Eigen::Vector2d sqrt_information_;
    Eigen::Matrix3d k_;
};

} // namespace hityavie