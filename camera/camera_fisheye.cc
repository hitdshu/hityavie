#include "camera/camera_fisheye.h"

namespace hityavie {

HITYAVIE_REGISTER_CAMERA(CameraFisheye);

bool CameraFisheye::Init(const CameraParam &param) {
    if (!param.has_fisheye_param()) {
        std::cout << "Does not have fisheye param, fisheye camera initialization fails!" << std::endl;
        return false;
    }
    if (param.tic_size() != 16) {
        std::cout << "Does not have extrinsic param, fisheye camera initialization fails!" << std::endl;
        return false;
    }
    tic_ << param.tic(0), param.tic(1), param.tic(2), param.tic(3),
        param.tic(4), param.tic(5), param.tic(6), param.tic(7),
        param.tic(8), param.tic(9), param.tic(10), param.tic(11),
        param.tic(12), param.tic(13), param.tic(14), param.tic(15);
    param_ = param.fisheye_param();
    k_ = cv::Mat(3, 3, CV_32FC1, cv::Scalar(0));
    k_.at<float>(0, 0) = param_.fx();
    k_.at<float>(0, 2) = param_.cx();
    k_.at<float>(1, 1) = param_.fy();
    k_.at<float>(1, 2) = param_.cy();
    k_.at<float>(2, 2) = 1;
    d_ = cv::Mat(4, 1, CV_32FC1, cv::Scalar(0));
    for (int idx = 0; idx < param_.dist_size(); ++idx) {
        d_.at<float>(idx, 0) = param_.dist(idx);
    }
    cv::Size img_size(param_.width(), param_.height());
    mapx_ = cv::Mat(param_.height(), param_.width(), CV_32FC1, cv::Scalar(0));
    mapy_ = cv::Mat(param_.height(), param_.width(), CV_32FC1, cv::Scalar(0));
    cv::fisheye::initUndistortRectifyMap(k_, d_, cv::Mat::eye(3, 3, CV_32FC1), k_, img_size, CV_32FC1, mapx_, mapy_);
    return true;
}

bool CameraFisheye::PreProcess(cv::Mat &img) const {
    cv::remap(img, img, mapx_, mapy_, CV_INTER_LINEAR);
    return true;
}

Eigen::Vector2d CameraFisheye::UndistortPt(const Eigen::Vector2d &pt) const {
    std::vector<cv::Point2f> pts;
    std::vector<cv::Point2f> und_pts;
    pts.push_back(cv::Point2f(pt[0], pt[1]));
    cv::fisheye::undistortPoints(pts, und_pts, k_, d_, cv::Mat::eye(3, 3, CV_32FC1), k_);
    Eigen::Vector2d und_pt;
    und_pt[0] = und_pts[0].x;
    und_pt[1] = und_pts[0].y;
    return und_pt;
}

std::vector<Eigen::Vector2d> CameraFisheye::UndistortPts(const std::vector<Eigen::Vector2d> &pts) const {
    std::vector<cv::Point2f> ori_pts;
    std::vector<cv::Point2f> und_pts;
    for (size_t idx = 0; idx < pts.size(); ++idx) {
        ori_pts.push_back(cv::Point2f(pts[idx][0], pts[idx][1]));
    }
    cv::fisheye::undistortPoints(ori_pts, und_pts, k_, d_);
    std::vector<Eigen::Vector2d> result;
    for (size_t idx = 0; idx < und_pts.size(); ++idx) {
        result.push_back(Eigen::Vector2d(und_pts[idx].x, und_pts[idx].y));
    }
    return result;
}

Eigen::Vector3d CameraFisheye::Pt2Ray(const Eigen::Vector2d &pt) const {
    Eigen::Vector3d ray;
    ray[0] = (pt[0] - param_.cx()) / param_.fx();
    ray[1] = (pt[1] - param_.cy()) / param_.fy();
    ray[2] = 1;
    return ray;
}

Eigen::Vector2d CameraFisheye::Project(const Eigen::Vector3d &pc) const {
    Eigen::Vector2d pt;
    pt[0] = param_.fx() * pc[0] / pc[2] + param_.cx();
    pt[1] = param_.fy() * pc[1] / pc[2] + param_.cy();
    return pt;
}

Eigen::Matrix<double, 2, 3> CameraFisheye::Jacobian(const Eigen::Vector3d &pc) const {
    Eigen::Matrix<double, 2, 3> jacob;
    jacob(0, 0) = param_.fx() / pc[2];
    jacob(0, 1) = 0;
    jacob(0, 2) = - param_.fx() * pc[0] / (pc[2] * pc[2]);
    jacob(1, 0) = 0;
    jacob(1, 1) = param_.fy() / pc[2];
    jacob(1, 2) = - param_.fy() * pc[1] / (pc[2] * pc[2]);
    return jacob;
}

std::string CameraFisheye::Name() const {
    return "CameraFisheye";
}

Eigen::Matrix4d CameraFisheye::GetTfic() const {
    return tic_;
}

Eigen::Matrix3d CameraFisheye::GetRic() const {
    return tic_.block(0, 0, 3, 3);
}

Eigen::Vector3d CameraFisheye::GetTic() const {
    return tic_.block(0, 3, 3, 1);
}

} // namespace hityavie