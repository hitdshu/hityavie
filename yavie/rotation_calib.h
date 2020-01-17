#pragma once

#include <vector>
#include <Eigen/Dense>
#include <memory>

namespace hityavie {

class RotationCalib {
public:
    typedef std::shared_ptr<RotationCalib> Ptr;

    RotationCalib();
    ~RotationCalib() = default;

    bool Calibrate(const Eigen::Matrix3d &rcpcc, const Eigen::Matrix3d &rbpbc);

    Eigen::Matrix3d GetRic() const {
        return rbc_;
    }
    bool IsDone() const {
        return calib_done_;
    }

    RotationCalib(const RotationCalib &) = delete;
    RotationCalib &operator=(const RotationCalib &) = delete;

private:
    std::vector<Eigen::Matrix3d> rck_ckps_;
    std::vector<Eigen::Matrix3d> rbk_bkps_;
    bool calib_done_;
    Eigen::Matrix3d rbc_;
};

} // namespace hityavie