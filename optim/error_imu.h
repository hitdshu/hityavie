#pragma once

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "imu/preintegrator.h"

namespace hityavie {

class ErrorImu : public ceres::SizedCostFunction<15, 7, 9, 7, 9> {
public:
    ErrorImu() = delete;
    ErrorImu(const Preintegrator::Ptr &pi);

    virtual bool Evaluate(double const * const *parameters, double *residuals, double **jacobians) const override;

private:
    Preintegrator::Ptr pi_;
};

} // namespace hityavie