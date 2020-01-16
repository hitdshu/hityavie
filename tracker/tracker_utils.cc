#include <Eigen/Dense>
#include "tracker/tracker_utils.h"

namespace hityavie {

std::vector<cv::KeyPoint> TrackerUtils::KeyPtsNms(const std::vector<cv::KeyPoint> &pts, double dist_thre) {
    if (pts.size() == 0) {
        return pts;
    }
    std::vector<bool> save_flag(pts.size(), true);
    std::vector<cv::KeyPoint> nms_pts;
    for (size_t idx1 = 0; idx1 < pts.size() - 1; ++idx1) {
        if (!save_flag[idx1]) {
            continue;
        }
        for (size_t idx2 = idx1 + 1; idx2 < pts.size(); ++idx2) {
            if (!save_flag[idx2]) {
                continue;
            }
            Eigen::Vector2d diff;
            diff << pts[idx1].pt.x - pts[idx2].pt.x, pts[idx1].pt.y - pts[idx2].pt.y;
            if (diff.norm() < dist_thre) {
                if (pts[idx1].response >= pts[idx2].response) {
                    save_flag[idx2] = false;
                } else {
                    save_flag[idx1] = false;
                }
            }
        }
    }
    for (size_t idx = 0; idx < pts.size(); ++idx) {
        if (save_flag[idx]) {
            nms_pts.push_back(pts[idx]);
        }
    }
    return nms_pts;
}

std::vector<Feature> TrackerUtils::KeyPtsNms(const std::vector<Feature> &pts, double dist_thre) {
    if (pts.size() == 0) {
        return pts;
    }
    std::vector<bool> save_flag(pts.size(), true);
    std::vector<Feature> nms_pts;
    for (size_t idx1 = 0; idx1 < pts.size() - 1; ++idx1) {
        if (!save_flag[idx1]) {
            continue;
        }
        for (size_t idx2 = idx1 + 1; idx2 < pts.size(); ++idx2) {
            if (!save_flag[idx2]) {
                continue;
            }
            Eigen::Vector2d diff = pts[idx1].pt_ori - pts[idx2].pt_ori;
            if (diff.norm() < dist_thre) {
                if (pts[idx1].response >= pts[idx2].response) {
                    save_flag[idx2] = false;
                } else {
                    save_flag[idx1] = false;
                }
            }
        }
    }
    for (size_t idx = 0; idx < pts.size(); ++idx) {
        if (save_flag[idx]) {
            nms_pts.push_back(pts[idx]);
        }
    }
    return nms_pts;
}

std::vector<cv::KeyPoint> TrackerUtils::MaxNPts(const std::vector<cv::KeyPoint> &pts, int n) {
    if (n >= pts.size()) {
        return pts;
    } else {
        return std::vector<cv::KeyPoint>(pts.begin(), pts.begin() + n);
    }
}

bool TrackerUtils::CompareResponse(const cv::KeyPoint &pt1, const cv::KeyPoint &pt2) {
    return pt1.response > pt2.response;
}

Eigen::Vector2d TrackerUtils::Pt2Vector(const cv::KeyPoint &pt) {
    return Eigen::Vector2d(pt.pt.x, pt.pt.y);
}

Eigen::Vector2d TrackerUtils::Pt2Vector(const cv::Point2f &pt) {
    return Eigen::Vector2d(pt.x, pt.y);
}

cv::Point2f TrackerUtils::Vector2Pt(const Eigen::Vector2d &vec) {
    return cv::Point2f(vec[0], vec[1]);
}

cv::Point2f TrackerUtils::Vector2Pt(const Feature &kp) {
    return cv::Point2f(kp.pt_ori[0], kp.pt_ori[1]);
}

bool TrackerUtils::PtNbd(const Eigen::Vector2d &pt, int nrows, int ncols, double bd_dist) {
    if (pt[0] < bd_dist || pt[0] > ncols - bd_dist) {
        return true;
    } 
    if (pt[1] < bd_dist || pt[1] > nrows - bd_dist) {
        return true;
    }
    return false;
}

std::vector<Feature> TrackerUtils::FilterByBd(const std::vector<Feature> &pts, int nrows, int ncols, double bd_dist) {
    std::vector<Feature> fpts;
    for (size_t idx = 0; idx < pts.size(); ++idx) {
        if (!PtNbd(pts[idx].pt_ori, nrows, ncols, bd_dist)) {
            fpts.push_back(pts[idx]);
        }
    }
    return fpts;
}

std::vector<cv::KeyPoint> TrackerUtils::FilterByBd(const std::vector<cv::KeyPoint> &pts, int nrows, int ncols, double bd_dist) {
    std::vector<cv::KeyPoint> fpts;
    for (size_t idx = 0; idx < pts.size(); ++idx) {
        Eigen::Vector2d pt;
        pt << pts[idx].pt.x, pts[idx].pt.y;
        if (!PtNbd(pt, nrows, ncols, bd_dist)) {
            fpts.push_back(pts[idx]);
        }
    }
    return fpts;
}

std::vector<Feature> TrackerUtils::FilterByCond(const std::vector<Feature> &pts, const std::vector<uchar> &flags) {
    std::vector<Feature> fpts;
    for (size_t idx = 0; idx < pts.size(); ++idx) {
        if (flags[idx]) {
            fpts.push_back(pts[idx]);
        }
    }
    return fpts;
}

} // namespace hityavie