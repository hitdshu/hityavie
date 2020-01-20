#pragma once

#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace hityavie {

class PoseParameterization : public ceres::LocalParameterization {
public:
    virtual bool Plus(const double *x, const double *delta, double *x_plus) const override;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const override;
    virtual int GlobalSize() const override { 
        return 7; 
    }
    virtual int LocalSize() const override { 
        return 6; 
    }
};

} // namespace hityavie