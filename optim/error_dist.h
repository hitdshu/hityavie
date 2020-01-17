#pragma once

#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace hityavie {

class ErrorDist {
public:
    ErrorDist(double dist, double sqrt_info) {
        dist_ = dist;
        sqrt_info_ = sqrt_info;
    }

    template <typename T>
    bool operator()(const T* const t_cw, T* residuals) const {
        T target_dist(dist_);
        T cur_dist = ceres::sqrt(t_cw[0] * t_cw[0] + t_cw[1] * t_cw[1] + t_cw[2] * t_cw[2]);
        residuals[0] = (cur_dist - target_dist) * T(sqrt_info_);
        return true;
    }

    static ceres::CostFunction* Create(double dist, double sqrt_info) {
        return (new ceres::AutoDiffCostFunction<ErrorDist, 1, 3>(new ErrorDist(dist, sqrt_info)));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    double dist_;
    double sqrt_info_;
};

} // namespace hityavie