#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <pangolin/pangolin.h>

namespace hityavie {

class Adapter {
public:
    static cv::Point2f Vec2Pt(const Eigen::Vector2d &v);
    static cv::Point3f Vec2Pt(const Eigen::Vector3d &v);
    static Eigen::Vector2d Pt2Vec(const cv::Point2f &pt);
    static Eigen::Vector3d Pt2Vec(const cv::Point3f &pt);
    static pangolin::OpenGlMatrix EigenMat2Pangolin(const Eigen::Matrix4d &t);

    Adapter() = delete;
};

} // namespace hityavie