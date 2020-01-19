#pragma once

#include <memory>
#include <Eigen/Dense>
#include <map>
#include "tracker/tracker_base.h"
#include "imu/preintegrator.h"

namespace hityavie {

class YavieFrame {
public:
    typedef std::shared_ptr<YavieFrame> Ptr;

    YavieFrame(double timestamp, const Eigen::Matrix4d &Twb, const std::vector<Feature> &feats, const Preintegrator::Ptr &pintor);
    ~YavieFrame() = default;

    void EnableObs(int id);
    void DisableObs(int id);
    void SetPose(const Eigen::Matrix4d &twb);

    int GetEffObsNum() const;
    bool IsEffeObs(int id) const;
    bool HasFeature(int id) const;
    std::vector<Feature> GetFeatures() const;
    Feature GetFeature(int fid) const;
    Eigen::Matrix4d GetPose() const;
    int GetId() const;

    YavieFrame(const YavieFrame &) = delete;
    YavieFrame &operator=(const YavieFrame &) = delete;

private:
    static int next_id_;
    int id_;
    double timestamp_;
    Eigen::Matrix4d twb_;
    std::map<int, Feature> features_;
    std::map<int, bool> iseff_;
    Preintegrator::Ptr pintor_;
};

} // namespace hityavie