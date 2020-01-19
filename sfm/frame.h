#pragma once

#include <map>
#include <Eigen/Dense>
#include "tracker/tracker_base.h"

namespace hityavie {

class Frame {
public:
    typedef std::shared_ptr<Frame> Ptr;

    Frame(const std::vector<Feature> &feats);
    ~Frame() = default;

    void EnableObs(int id);
    void DisableObs(int id);
    void SetPose(const Eigen::Matrix4d &tcw) {
        tcw_ = tcw;
    }

    int GetEffObsNum() const;
    bool IsEffeObs(int id) const;
    std::vector<Feature> GetFeatures() const;
    Feature GetFeature(int fid) const;
    Eigen::Matrix4d GetPose() const {
        return tcw_;
    }
    int GetId() const {
        return id_;
    }

    Frame(const Frame &) = delete;
    Frame &operator=(const Frame &) = delete;

private:
    static int next_id_;

    int id_;
    std::map<int, Feature> features_;
    std::map<int, bool> iseff_;
    Eigen::Matrix4d tcw_;
};

} // namespace hityavie