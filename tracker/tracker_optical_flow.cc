#include <algorithm>
#include "tracker/tracker_optical_flow.h"
#include "tracker/tracker_utils.h"

namespace hityavie {

HITYAVIE_REGISTER_TRACKER(TrackerOpticalFlow);

bool TrackerOpticalFlow::Init(const TrackerParam &param, std::shared_ptr<BaseCamera> camera) {
    if (!param.has_of_param()) {
        std::cout << "Tracker param does not have of_param, TrackerOpticalFlow init fails!" << std::endl;
        return false;
    }
    param_ = param.of_param();
    camera_ = camera;
    next_kp_id_ = 0;
    pts_lf_.clear();
    img_lf_ = cv::Mat();
    feat_dtr_ = cv::FastFeatureDetector::create(param_.det_thre());
    of_tracker_ = cv::SparsePyrLKOpticalFlow::create();
}

bool TrackerOpticalFlow::Track(cv::Mat &img, std::vector<Feature> &pts) {
    pts.clear();
    if (img_lf_.empty()) {
        std::vector<cv::KeyPoint> kpts;
        feat_dtr_->detect(img, kpts);
        std::sort(kpts.begin(), kpts.end(), TrackerUtils::CompareResponse);
        kpts = TrackerUtils::FilterByBd(kpts, img.rows, img.cols, param_.bd_thre());
        kpts = TrackerUtils::KeyPtsNms(kpts, param_.nms_dist_thre());
        kpts = TrackerUtils::MaxNPts(kpts, param_.min_feat_num() * 2);
        for (const auto &pt : kpts) {
            Feature tmp;
            tmp.id = next_kp_id_++;
            tmp.response = pt.response;
            tmp.pt_ori = TrackerUtils::Pt2Vector(pt);
            tmp.pt_und = camera_->UndistortPt(tmp.pt_ori);
            pts.push_back(tmp);
        }
    } else {
        std::vector<cv::Point2f> prev_pts;
        std::vector<cv::Point2f> next_pts;
        std::vector<uchar> status;
        for (size_t idx = 0; idx < pts_lf_.size(); ++idx) {
            prev_pts.push_back(TrackerUtils::Vector2Pt(pts_lf_[idx]));
        }
        of_tracker_->calc(img_lf_, img, prev_pts, next_pts, status);
        std::vector<cv::Point2f> prev_fund_pts;
        std::vector<cv::Point2f> next_fund_pts;
        std::vector<uchar> fund_status;
        for (size_t idx = 0; idx < pts_lf_.size(); ++idx) {
            if (status[idx]) {
                Feature tmp;
                tmp.id = pts_lf_[idx].id;
                tmp.pt_ori = TrackerUtils::Pt2Vector(next_pts[idx]);
                tmp.pt_und = camera_->UndistortPt(tmp.pt_ori);
                prev_fund_pts.push_back(TrackerUtils::Vector2Pt(pts_lf_[idx].pt_und));
                next_fund_pts.push_back(TrackerUtils::Vector2Pt(tmp.pt_und));
                pts.push_back(tmp);
            }
        }
        cv::findFundamentalMat(prev_fund_pts, next_fund_pts, cv::FM_RANSAC, 2, 0.99, fund_status);
        pts = TrackerUtils::FilterByCond(pts, fund_status);
        pts = TrackerUtils::KeyPtsNms(pts, param_.nms_dist_thre() / 4.0);
        if (pts.size() < param_.min_feat_num()) {
            cv::Mat mask(img.rows, img.cols, CV_8UC1, cv::Scalar(1));
            for (size_t idx = 0; idx < pts.size(); ++idx) {
                cv::circle(mask, TrackerUtils::Vector2Pt(pts[idx].pt_ori), param_.nms_dist_thre(), cv::Scalar(0), -1);
            }
            std::vector<cv::KeyPoint> kpts;
            feat_dtr_->detect(img, kpts, mask);
            std::sort(kpts.begin(), kpts.end(), TrackerUtils::CompareResponse);
            kpts = TrackerUtils::FilterByBd(kpts, img.rows, img.cols, param_.bd_thre());
            kpts = TrackerUtils::KeyPtsNms(kpts, param_.nms_dist_thre());
            kpts = TrackerUtils::MaxNPts(kpts, param_.min_feat_num());
            for (const auto &pt : kpts) {
                Feature tmp;
                tmp.id = next_kp_id_++;
                tmp.response = pt.response;
                tmp.pt_ori = TrackerUtils::Pt2Vector(pt);
                tmp.pt_und = camera_->UndistortPt(tmp.pt_ori);
                pts.push_back(tmp);
            }
        }
    }
    pts = TrackerUtils::FilterByBd(pts, img.rows, img.cols, param_.bd_thre());
    img_lf_ = img;
    pts_lf_ = pts;
    return pts.size() >= param_.min_feat_num();
}

} // namespace hityavie