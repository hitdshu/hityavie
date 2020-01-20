#include "optim/error_prj_vie.h"
#include "common/geometry_utility.h"

namespace hityavie {

bool ErrorPrjVie::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d pwb(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond qwb(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Vector3d pbc(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond qbc(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d pw(parameters[2][0], parameters[2][1], parameters[2][2]);

    Eigen::Vector3d pb = qwb.inverse() * (pw - pwb);
    Eigen::Vector3d pc = qbc.inverse() * (pb - pbc);
    Eigen::Vector2d prj;
    double fx = k_(0, 0);
    double fy = k_(1, 1);
    double cx = k_(0, 2);
    double cy = k_(1, 2);
    prj[0] = fx * pc[0] / pc[2] + cx;
    prj[1] = fy * pc[1] / pc[2] + cy;
    Eigen::Map<Eigen::Vector2d> residual_vec(residuals);
    residual_vec = prj - p_obs_;
    residual_vec = sqrt_information_ * residual_vec;

    if (jacobians) {
        Eigen::Matrix<double, 2, 3> jacob_prj;
        jacob_prj(0, 0) = fx / pc[2];
        jacob_prj(0, 1) = 0;
        jacob_prj(0, 2) = -fx * pc[0] / (pc[2] * pc[2]);
        jacob_prj(1, 0) = 0;
        jacob_prj(1, 1) = fy / pc[2];
        jacob_prj(1, 2) = -fy * pc[1] / (pc[2] * pc[2]);
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_twb(jacobians[0]);
            jacobian_twb.setZero();
            Eigen::Matrix<double, 3, 6> jacob_pc_twb;
            jacob_pc_twb.leftCols<3>() = -qbc.inverse().toRotationMatrix() * qwb.inverse().toRotationMatrix();
            jacob_pc_twb.rightCols<3>() = qbc.inverse().toRotationMatrix() * GeometryUtility::Skew(pb);
            jacobian_twb.leftCols<6>() = sqrt_information_ * jacob_prj * jacob_pc_twb;
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_tbc(jacobians[1]);
            jacobian_tbc.setZero();
            Eigen::Matrix<double, 3, 6> jacob_pc_tbc;
            jacob_pc_tbc.leftCols<3>() = -qbc.inverse().toRotationMatrix();
            jacob_pc_tbc.rightCols<3>() = GeometryUtility::Skew(pc);
            jacobian_tbc.leftCols<6>() = sqrt_information_ * jacob_prj * jacob_pc_tbc;
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_pw(jacobians[2]);
            jacobian_pw.setZero();
            Eigen::Matrix<double, 3, 3> jacob_pc_pw;
            jacob_pc_pw = qbc.inverse().toRotationMatrix() * qwb.inverse().toRotationMatrix();
            jacobian_pw = sqrt_information_ * jacob_prj * jacob_pc_pw;
        }
    }
    return true;
}

} // namespace hityavie