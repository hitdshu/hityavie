#include "common/drawer.h"

namespace hityavie {

void Drawer::DrawPts(cv::Mat &img, const std::vector<Feature> &pts, bool draw_ori, const cv::Scalar &color) {
    for (const auto &pt : pts) {
        cv::Point2f cvpt;
        if (draw_ori) {
            cvpt = cv::Point2f(pt.pt_ori[0], pt.pt_ori[1]);
        } else {
            cvpt = cv::Point2f(pt.pt_und[0], pt.pt_und[1]);
        }
        cv::circle(img, cvpt, 1, color);
    }
    return;
}

void Drawer::DrawPts(cv::Mat &img, const std::vector<Eigen::Vector2d> &pts, const cv::Scalar &color) {
    for (const auto &pt : pts) {
        cv::circle(img, cv::Point2f(pt[0], pt[1]), 1, color);
    }
    return;
}

void Drawer::DrawPtsTraj(cv::Mat &img, const std::vector<Eigen::Vector2d> &pts_prev, const std::vector<Eigen::Vector2d> &pts_cur, const cv::Scalar &color) {
    for (size_t idx = 0; idx < pts_prev.size(); ++idx) {
        cv::line(img, cv::Point2f(pts_prev[idx][0], pts_prev[idx][1]), cv::Point2f(pts_cur[idx][0], pts_cur[idx][1]), color);
    }
    return;
}

} // namespace hityavie