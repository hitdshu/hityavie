#pragma once

#include <opencv2/opencv.hpp>
#include "tracker/tracker_base.h"

namespace hityavie {

class Drawer {
public:
    static void DrawPts(cv::Mat &img, const std::vector<Feature> &pts, bool draw_ori = true, const cv::Scalar &color = cv::Scalar(0, 0, 255));
    static void DrawPts(cv::Mat &img, const std::vector<Eigen::Vector2d> &pts, const cv::Scalar &color = cv::Scalar(0, 0, 255));
    static void DrawPtsTraj(cv::Mat &img, const std::vector<Eigen::Vector2d> &pts_prev, const std::vector<Eigen::Vector2d> &pts_cur, const cv::Scalar &color = cv::Scalar(0, 0, 255));
};

} // namespace hityavie