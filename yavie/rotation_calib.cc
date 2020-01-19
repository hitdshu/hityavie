#include "yavie/rotation_calib.h"
#include "common/geometry_utility.h"
#include "common/adapter.h"

namespace hityavie {

RotationCalib::RotationCalib() {
    calib_done_ = false;
    rbc_.setIdentity();
}

bool RotationCalib::Calibrate(const Eigen::Matrix3d &rcpcc, const Eigen::Matrix3d &rbpbc) {
    rck_ckps_.push_back(rcpcc);
    rbk_bkps_.push_back(rbpbc);
    if (rck_ckps_.size() < 3) {
        return false;
    }
    Eigen::MatrixXd A(rck_ckps_.size() * 4, 4);
    for (size_t idx = 0; idx < rck_ckps_.size(); ++idx) {
        Eigen::Quaterniond qb(rbk_bkps_[idx]);
        Eigen::Quaterniond qc(rck_ckps_[idx]);
        Eigen::Quaterniond qcfb(rbc_.inverse() * qb.toRotationMatrix() * rbc_);
        double ang_dist = 180 / CV_PI * qc.angularDistance(qcfb);
        double huber = ang_dist > 5.0 ? 5.0 / ang_dist : 1.0;
        A.block(idx * 4, 0, 4, 4) = (GeometryUtility::QuatLmat(qb) - GeometryUtility::QuatRmat(qc)) * huber;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector4d v = svd.matrixV().col(3);
    Eigen::Matrix3d rbc = Adapter::Quat2Rot(v);
    rbc_ = rbc;
    if (svd.singularValues()[2] > 0.05) {
        calib_done_ = true;
    }
    return calib_done_;
}

} // namespace hityavie