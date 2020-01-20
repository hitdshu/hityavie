#include "optim/error_imu.h"
#include "common/geometry_utility.h"

namespace hityavie {

namespace {

Eigen::Matrix4d Qleft(const Eigen::Quaterniond &q) {
    Eigen::Matrix4d ql;
    ql(0, 0) = q.w();
    ql.template block<1, 3>(0, 1) = -q.vec().transpose();
    ql.template block<3, 1>(1, 0) = q.vec();
    ql.template block<3, 3>(1, 1) = q.w() * Eigen::Matrix3d::Identity() + GeometryUtility::Skew(q.vec());
    return ql;
}

Eigen::Matrix4d Qright(const Eigen::Quaterniond &p) {
    Eigen::Matrix4d qr;
    qr(0, 0) = p.w();
    qr.template block<1, 3>(0, 1) = -p.vec().transpose();
    qr.template block<3, 1>(1, 0) = p.vec();
    qr.template block<3, 3>(1, 1) = p.w() * Eigen::Matrix3d::Identity() - GeometryUtility::Skew(p.vec());
    return qr;
}

} // namespace

ErrorImu::ErrorImu(const Preintegrator::Ptr &pi) {
    pi_ = pi;
}

bool ErrorImu::Evaluate(double const * const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Vector3d vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d bai(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d bgi(parameters[1][6], parameters[1][7], parameters[1][8]);
    Eigen::Vector3d pj(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
    Eigen::Vector3d vj(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Vector3d baj(parameters[3][3], parameters[3][4], parameters[3][5]);
    Eigen::Vector3d bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

    Eigen::Map<Eigen::Matrix<double, 15, 1>> residual_vec(residuals);
    residual_vec = pi_->Evaluate(pi, qi, vi, bai, bgi, pj, qj, vj, baj, bgj);
    Eigen::Matrix<double, 15, 15> sqrt_info = pi_->GetCovariance().inverse().llt().matrixL().transpose();
    residual_vec = sqrt_info * residual_vec;

    if (jacobians) {
        double dt = pi_->GetDt();
        Eigen::Vector3d g = pi_->GetGravity();
        Eigen::Matrix<double, 15, 15> pi_jacob = pi_->GetJacobian();
        Eigen::Matrix3d dpdba = pi_jacob.block(0, 9, 3, 3);
        Eigen::Matrix3d dpdbg = pi_jacob.block(0, 12, 3, 3);
        Eigen::Matrix3d dvdba = pi_jacob.block(3, 9, 3, 3);
        Eigen::Matrix3d dvdbg = pi_jacob.block(3, 12, 3, 3);
        Eigen::Matrix3d dqdbg = pi_jacob.block(6, 12, 3, 3);
        Eigen::Quaterniond corrected_delta_q = pi_->GetQ() * GeometryUtility::AngleAxis2Quat(dqdbg * (bgi - pi_->GetBg()));
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            jacobian_pose_i.setZero();
            jacobian_pose_i.block(0, 0, 3, 3) = -qi.inverse().toRotationMatrix();
            jacobian_pose_i.block(0, 3, 3, 3) = GeometryUtility::Skew(qi.inverse() * (0.5 * g * dt * dt + pj - pi - vi * dt));
            jacobian_pose_i.block(3, 3, 3, 3) = GeometryUtility::Skew(qi.inverse() * (g * dt + vj - vi));
            jacobian_pose_i.block(6, 3, 3, 3) = -(Qleft(qj.inverse() * qi) * Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
            jacobian_pose_i = sqrt_info * jacobian_pose_i;
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_vbabg_i(jacobians[1]);
            jacobian_vbabg_i.setZero();
            jacobian_vbabg_i.block<3, 3>(0, 0) = -qi.inverse().toRotationMatrix() * dt;
            jacobian_vbabg_i.block<3, 3>(0, 3) = -dpdba;
            jacobian_vbabg_i.block<3, 3>(0, 6) = -dpdbg;
            jacobian_vbabg_i.block<3, 3>(3, 0) = -qi.inverse().toRotationMatrix();
            jacobian_vbabg_i.block<3, 3>(3, 3) = -dvdba;
            jacobian_vbabg_i.block<3, 3>(3, 6) = -dvdbg;
            jacobian_vbabg_i.block<3, 3>(6, 6) = -Qleft(qj.inverse() * qi * corrected_delta_q).bottomRightCorner<3, 3>() * dqdbg;
            jacobian_vbabg_i.block<3, 3>(9, 3) = -Eigen::Matrix3d::Identity();
            jacobian_vbabg_i.block<3, 3>(12, 6) = -Eigen::Matrix3d::Identity();
            jacobian_vbabg_i = sqrt_info * jacobian_vbabg_i;
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
            jacobian_pose_j.setZero();
            jacobian_pose_j.block<3, 3>(0, 0) = qi.inverse().toRotationMatrix();
            jacobian_pose_j.block<3, 3>(6, 3) = Qleft(corrected_delta_q.inverse() * qi.inverse() * qj).bottomRightCorner<3, 3>();
            jacobian_pose_j = sqrt_info * jacobian_pose_j;
        }
        if (jacobians[3]) {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_vbabg_j(jacobians[3]);
            jacobian_vbabg_j.setZero();
            jacobian_vbabg_j.block<3, 3>(3, 0) = qi.inverse().toRotationMatrix();
            jacobian_vbabg_j.block<3, 3>(9, 3) = Eigen::Matrix3d::Identity();
            jacobian_vbabg_j.block<3, 3>(12, 6) = Eigen::Matrix3d::Identity();
            jacobian_vbabg_j = sqrt_info * jacobian_vbabg_j;
        }
    }
    return true;
}

} // namespace hityavie