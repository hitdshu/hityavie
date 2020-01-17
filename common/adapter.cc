#include "common/adapter.h"

namespace hityavie {

cv::Point2f Adapter::Vec2Pt(const Eigen::Vector2d &v) {
    cv::Point2f pt(v[0], v[1]);
    return pt;
}

cv::Point3f Adapter::Vec2Pt(const Eigen::Vector3d &v) {
    cv::Point3f pt(v[0], v[1], v[2]);
    return pt;
}

Eigen::Vector3d Adapter::Pt2Vec(const cv::Point3f &pt) {
    Eigen::Vector3d v;
    v << pt.x, pt.y, pt.z;
    return v;
}

Eigen::Vector2d Adapter::Pt2Vec(const cv::Point2f &pt) {
    Eigen::Vector2d v;
    v << pt.x, pt.y;
    return v;
}

Eigen::Vector4d Adapter::Rot2Quat(const Eigen::Matrix3d &rot) {
    Eigen::Quaterniond quat(rot);
    Eigen::Vector4d quatv;
    quatv << quat.w(), quat.x(), quat.y(), quat.z();
    return quatv;
}

Eigen::Matrix3d Adapter::Quat2Rot(const Eigen::Vector4d &quat) {
    Eigen::Quaterniond q;
    q.w() = quat[0];
    q.x() = quat[1];
    q.y() = quat[2];
    q.z() = quat[3];
    return q.toRotationMatrix();
}

Eigen::Matrix3d Adapter::Cvk2Matk(const cv::Mat &kcv) {
    Eigen::Matrix3d k;
    k << kcv.at<float>(0, 0), kcv.at<float>(0, 1), kcv.at<float>(0, 2), 
        kcv.at<float>(1, 0), kcv.at<float>(1, 1), kcv.at<float>(1, 2), 
        kcv.at<float>(2, 0), kcv.at<float>(2, 1), kcv.at<float>(2, 2);
    return k;
}

pangolin::OpenGlMatrix Adapter::EigenMat2Pangolin(const Eigen::Matrix4d &t) {
    pangolin::OpenGlMatrix mat;
    mat.SetIdentity();
    mat.m[0] = t(0, 0);
    mat.m[1] = t(1, 0);
    mat.m[2] = t(2, 0);
    mat.m[3]  = 0.0;
    mat.m[4] = t(0, 1);
    mat.m[5] = t(1, 1);
    mat.m[6] = t(2, 1);
    mat.m[7]  = 0.0;
    mat.m[8] = t(0, 2);
    mat.m[9] = t(1, 2);
    mat.m[10] = t(2, 2);
    mat.m[11]  = 0.0;
    mat.m[12] = t(0, 3);
    mat.m[13] = t(1, 3);
    mat.m[14] = t(2, 3);
    mat.m[15]  = 1.0;
    return mat;
}

} // namespace hityavie