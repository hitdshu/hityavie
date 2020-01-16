#pragma once

#include "tracker/tracker_base.h"

namespace hityavie {

class TrackerOpticalFlow : public BaseTracker {
public:
    virtual bool Init(const TrackerParam &param, std::shared_ptr<BaseCamera> camera) override;
    virtual bool Track(cv::Mat &img, std::vector<Feature> &pts) override;

private:
    OpticalflowTrackerParam param_;
    std::shared_ptr<BaseCamera> camera_;
    cv::Ptr<cv::FastFeatureDetector> feat_dtr_;
    cv::Ptr<cv::SparsePyrLKOpticalFlow> of_tracker_;
    int next_kp_id_;
    std::vector<Feature> pts_lf_;
    cv::Mat img_lf_;
};

} // namespace hityavie