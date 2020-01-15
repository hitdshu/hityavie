#pragma once

#include <vector>
#include <Eigen/Dense>
#include "proto/param.pb.h"

namespace hityavie {

class Preintegrator {
public:
    Preintegrator() = default;
    ~Preintegrator() = default;

    void Init(const ImuNoiseParameter &np, const Eigen::Vector3d g, const Eigen::Vector3d &acc_init, const Eigen::Vector3d &gyr_init, const Eigen::Vector3d &ba_init, const Eigen::Vector3d &bg_init);
    void Integrate(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);
    void Reintegrate(const Eigen::Vector3d &ba_init, const Eigen::Vector3d &bg_init);
    Eigen::Matrix<double, 15, 1> Evaluate(const Eigen::Vector3d &p1, const Eigen::Quaterniond &q1, const Eigen::Vector3d &v1, const Eigen::Vector3d &ba1, const Eigen::Vector3d &bg1, 
        const Eigen::Vector3d &p2, const Eigen::Quaterniond &q2, const Eigen::Vector3d &v2, const Eigen::Vector3d &ba2, const Eigen::Vector3d &bg2);

    Preintegrator(const Preintegrator &) = delete;
    Preintegrator &operator=(const Preintegrator &) = delete;

protected:
    void Propagate(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);

private:
    Eigen::Vector3d p_;
    Eigen::Vector3d v_;
    Eigen::Quaterniond q_;
    Eigen::Vector3d ba_;
    Eigen::Vector3d bg_;
    Eigen::Matrix<double, 15, 15> jacob_;
    Eigen::Matrix<double, 15, 15> cov_;
    Eigen::Matrix<double, 15, 15> nd_cov_;
    Eigen::Vector3d gravity_;
    Eigen::Vector3d acc_init_;
    Eigen::Vector3d gyr_init_;
    Eigen::Vector3d acc_last_;
    Eigen::Vector3d gyr_last_;
    Eigen::Vector3d acc_cur_;
    Eigen::Vector3d gyr_cur_;
    std::vector<double> dt_buf_;
    std::vector<Eigen::Vector3d> acc_buf_;
    std::vector<Eigen::Vector3d> gyr_buf_;
};

} // namespace hityaview