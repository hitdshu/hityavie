#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "tracker/tracker_base.h"

namespace hityavie {

class TrackerUtils {
public:
    static std::vector<cv::KeyPoint> KeyPtsNms(const std::vector<cv::KeyPoint> &pts, double dist_thre);
    static std::vector<Feature> KeyPtsNms(const std::vector<Feature> &pts, double dist_thre);
    static std::vector<cv::KeyPoint> MaxNPts(const std::vector<cv::KeyPoint> &pts, int n);
    static bool CompareResponse(const cv::KeyPoint &pt1, const cv::KeyPoint &pt2);
    static Eigen::Vector2d Pt2Vector(const cv::KeyPoint &pt);
    static Eigen::Vector2d Pt2Vector(const cv::Point2f &pt);
    static cv::Point2f Vector2Pt(const Eigen::Vector2d &vec);
    static cv::Point2f Vector2Pt(const Feature &kp);
    static bool PtNbd(const Eigen::Vector2d &pt, int nrows, int ncols, double bd_dist);
    static std::vector<Feature> FilterByBd(const std::vector<Feature> &pts, int nrows, int ncols, double bd_dist);
    static std::vector<cv::KeyPoint> FilterByBd(const std::vector<cv::KeyPoint> &pts, int nrows, int ncols, double bd_dist);
    static std::vector<Feature> FilterByCond(const std::vector<Feature> &pts, const std::vector<uchar> &flags);
};

} // namespace hityavie