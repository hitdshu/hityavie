#include "imu/preintegrator.h"
#include "common/geometry_utility.h"

namespace hityavie {

void Preintegrator::Init(const ImuNoiseParameter &np, const Eigen::Vector3d &g, const Eigen::Vector3d &acc_init, const Eigen::Vector3d &gyr_init, const Eigen::Vector3d &ba_init, const Eigen::Vector3d &bg_init) {
    p_.setZero();
    v_.setZero();
    q_.setIdentity();
    ba_ = ba_init;
    bg_ = bg_init;
    jacob_.setIdentity();
    cov_.setZero();
    acc_init_ = acc_init;
    gyr_init_ = gyr_init;
    acc_last_ = acc_init;
    gyr_last_ = gyr_init;
    acc_cur_ = acc_init;
    gyr_cur_ = gyr_init;
    dt_buf_.clear();
    acc_buf_.clear();
    gyr_buf_.clear();
    nd_cov_.setZero();
    nd_cov_.block(3, 3, 3, 3) = np.acc_noise() * np.acc_noise() * Eigen::Matrix3d::Identity();
    nd_cov_.block(6, 6, 3, 3) = np.gyr_noise() * np.gyr_noise() * Eigen::Matrix3d::Identity();
    nd_cov_.block(9, 9, 3, 3) = np.ba_noise() * np.ba_noise() * Eigen::Matrix3d::Identity();
    nd_cov_.block(12, 12, 3, 3) = np.bg_noise() * np.bg_noise() * Eigen::Matrix3d::Identity();
    gravity_ = g;
}

void Preintegrator::Integrate(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr) {
    dt_buf_.push_back(dt);
    acc_buf_.push_back(acc);
    gyr_buf_.push_back(gyr);
    Propagate(dt, acc, gyr);
}

void Preintegrator::Reintegrate(const Eigen::Vector3d &ba_init, const Eigen::Vector3d &bg_init) {
    acc_last_ = acc_init_;
    gyr_last_ = gyr_init_;
    p_.setZero();
    v_.setZero();
    q_.setIdentity();
    ba_ = ba_init;
    bg_ = bg_init;
    jacob_.setIdentity();
    cov_.setZero();
    for (size_t idx = 0; idx < dt_buf_.size(); ++idx) {
        Propagate(dt_buf_[idx], acc_buf_[idx], gyr_buf_[idx]);
    }
}

Eigen::Matrix<double, 15, 1> Preintegrator::Evaluate(const Eigen::Vector3d &p1, const Eigen::Quaterniond &q1, const Eigen::Vector3d &v1, const Eigen::Vector3d &ba1, const Eigen::Vector3d &bg1, 
        const Eigen::Vector3d &p2, const Eigen::Quaterniond &q2, const Eigen::Vector3d &v2, const Eigen::Vector3d &ba2, const Eigen::Vector3d &bg2) {
    double total_dt = 0;
    for (const auto &dt : dt_buf_) {
        total_dt += dt;
    }
    Eigen::Matrix<double, 15, 1> residuals;
    Eigen::Matrix3d dpdba = jacob_.block(0, 9, 3, 3);
    Eigen::Matrix3d dpdbg = jacob_.block(0, 12, 3, 3);
    Eigen::Matrix3d dvdba = jacob_.block(3, 9, 3, 3);
    Eigen::Matrix3d dvdbg = jacob_.block(3, 12, 3, 3);
    Eigen::Matrix3d dqdba = jacob_.block(6, 9, 3, 3);
    Eigen::Matrix3d dqdbg = jacob_.block(6, 12, 3, 3);
    Eigen::Vector3d dba = ba1 - ba_;
    Eigen::Vector3d dbg = bg1 - bg_;
    Eigen::Vector3d pu = p_ + dpdba * dba + dpdbg * dbg;
    Eigen::Vector3d vu = v_ + dvdba * dba + dvdbg * dbg;
    Eigen::Quaterniond qu = q_ * GeometryUtility::AngleAxis2Quat(dqdba * dba + dqdbg * dbg);
    residuals.block(0, 0, 3, 1) = q1.inverse() * (0.5 * gravity_ * total_dt * total_dt + p2 - p1 - v1 * total_dt) - pu;
    residuals.block(3, 0, 3, 1) = q1.inverse() * (gravity_ * total_dt + v2 - v1) - vu;
    residuals.block(6, 0, 3, 1) = 2 * (qu.inverse() * (q1.inverse() * q2)).vec();
    residuals.block(9, 0, 3, 1) = ba2 - ba1;
    residuals.block(12, 0, 3, 1) = bg2 - bg1;
    return residuals;
}

void Preintegrator::Propagate(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr) {
    acc_cur_ = acc;
    gyr_cur_ = gyr;
    Eigen::Vector3d gyr_unb = (gyr_last_ + gyr_cur_) / 2.0 - bg_;
    Eigen::Quaterniond nq = q_ * GeometryUtility::AngleAxis2Quat(gyr_unb * dt);
    Eigen::Vector3d accw_last_unb = q_ * (acc_last_ - ba_);
    Eigen::Vector3d accw_cur_unb = nq * (acc_cur_ - ba_);
    Eigen::Vector3d accw_unb = (accw_last_unb + accw_cur_unb) / 2;
    Eigen::Vector3d nv = v_ + accw_unb * dt;
    Eigen::Vector3d np = p_ + v_ * dt + accw_unb * dt * dt / 2;
    Eigen::Vector3d acc_unb = (acc_last_ + acc_cur_) / 2 - ba_;
    Eigen::Matrix<double, 15, 15> err_state_djac;
    err_state_djac.setZero();
    err_state_djac.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
    err_state_djac.block(3, 6, 3, 3) = -q_.toRotationMatrix() * GeometryUtility::Skew(acc_unb);
    err_state_djac.block(3, 9, 3, 3) = -q_.toRotationMatrix();
    err_state_djac.block(6, 6, 3, 3) = -GeometryUtility::Skew(gyr_unb);
    err_state_djac.block(6, 12, 3, 3) = -Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 15, 15> err_state_jac;
    err_state_jac.setIdentity();
    err_state_jac += err_state_djac * dt;
    jacob_ = err_state_jac * jacob_;
    cov_ = err_state_jac * cov_ * err_state_jac.transpose() + nd_cov_ * dt;
    acc_last_ = acc_cur_;
    gyr_last_ = gyr_cur_;
    p_ = np;
    v_ = nv;
    q_ = nq;
}

Eigen::Vector3d Preintegrator::GetP() const {
    return p_;
}

Eigen::Vector3d Preintegrator::GetV() const {
    return v_;
}

Eigen::Quaterniond Preintegrator::GetQ() const {
    return q_;
}

Eigen::Vector3d Preintegrator::GetBa() const {
    return ba_;
}

Eigen::Vector3d Preintegrator::GetBg() const {
    return bg_;
}

Eigen::Matrix<double, 15, 15> Preintegrator::GetJacobian() const {
    return jacob_;
}

Eigen::Matrix<double, 15, 15> Preintegrator::GetCovariance() const {
    return cov_;
}

Eigen::Vector3d Preintegrator::GetGravity() const {
    return gravity_;
}

double Preintegrator::GetDt() const {
    double total_dt = 0;
    for (const auto &dt : dt_buf_) {
        total_dt += dt;
    }
    return total_dt;
}

} // namespace hityavie