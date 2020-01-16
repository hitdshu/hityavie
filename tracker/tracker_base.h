#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "common/register.h"
#include "camera/camera_base.h"
#include "proto/param.pb.h"

namespace hityavie {

struct Feature {
    int id;
    double response;
    Eigen::Vector2d pt_ori;
    Eigen::Vector2d pt_und;
};

class BaseTracker {
public:
    typedef std::shared_ptr<BaseTracker> Ptr;

    BaseTracker() = default;
    virtual ~BaseTracker() = default;

    virtual bool Init(const TrackerParam &param, std::shared_ptr<BaseCamera> camera) = 0;
    virtual bool Track(cv::Mat &img, std::vector<Feature> &pts) = 0;

    BaseTracker(const BaseTracker &) = delete;
    BaseTracker &operator=(const BaseTracker &) = delete;
};

HITYAVIE_REGISTER_REGISTERER(BaseTracker);
#define HITYAVIE_REGISTER_TRACKER(name) \
    HITYAVIE_REGISTER_CLASS(BaseTracker, name)

} // namespace hityavie