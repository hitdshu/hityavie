#include "yavie/yavie_frame.h"

namespace hityavie {

int YavieFrame::next_id_ = 0;

YavieFrame::YavieFrame(double timestamp, const Eigen::Matrix4d &Twb, const std::vector<Feature> &feats, const Preintegrator::Ptr &pintor) {
    id_ = next_id_++;
    for (const auto &feat : feats) {
        features_[feat.id] = feat;
        iseff_[feat.id] = false;
    }
    timestamp_ = timestamp;
    twb_ = Twb;
    pintor_ = pintor;
}

void YavieFrame::EnableObs(int id) {
    auto iter = features_.find(id);
    if (iter != features_.end()) {
        iseff_[iter->first] = true;
    }
}

void YavieFrame::DisableObs(int id) {
    auto iter = features_.find(id);
    if (iter != features_.end()) {
        iseff_[iter->first] = false;
    }
}

void YavieFrame::SetPose(const Eigen::Matrix4d &twb) {
    twb_ = twb;
}

int YavieFrame::GetEffObsNum() const {
    int cnt = 0;
    for (auto iter = iseff_.begin(); iter != iseff_.end(); ++iter) {
        if (iter->second) {
            cnt++;
        }
    }
    return cnt;
}

bool YavieFrame::IsEffeObs(int id) const {
    auto iter = iseff_.find(id);
    if (iter != iseff_.end()) {
        return iter->second;
    } else {
        return false;
    }
}

bool YavieFrame::HasFeature(int id) const {
    auto iter = iseff_.find(id);
    if (iter != iseff_.end()) {
        return true;
    } else {
        return false;
    }
}

std::vector<Feature> YavieFrame::GetFeatures() const {
    std::vector<Feature> feats;
    for (auto iter = features_.begin(); iter != features_.end(); ++iter) {
        feats.push_back(iter->second);
    }
    return feats;
}

Feature YavieFrame::GetFeature(int fid) const {
    auto iter = features_.find(fid);
    return iter->second;
}

Eigen::Matrix4d YavieFrame::GetPose() const {
    return twb_;
}

int YavieFrame::GetId() const {
    return id_;
}

} // namespace hityavie