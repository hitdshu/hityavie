#pragma once

#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace hityavie {

class ErrorPrjVie : public ceres::SizedCostFunction<2, 7, 7, 3> {
public:
    ErrorPrjVie(const Eigen::Vector2d &p_obs, const Eigen::Vector2d &sqrt_information, const Eigen::Matrix3d &k) {
        p_obs_ = p_obs;
        sqrt_information_.setZero();
        sqrt_information_(0, 0) = sqrt_information[0];
        sqrt_information_(1, 1) = sqrt_information[1];
        k_ = k;
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;

private:
    Eigen::Vector2d p_obs_;
    Eigen::Matrix2d sqrt_information_;
    Eigen::Matrix3d k_;
};

} // namespace hityavie