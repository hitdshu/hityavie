#include "sfm/frame.h"

namespace hityavie {

int Frame::next_id_ = 0;

Frame::Frame(const std::vector<Feature> &feats) {
    id_ = next_id_++;
    for (const auto &feat : feats) {
        features_[feat.id] = feat;
        iseff_[feat.id] = false;
    }
}

void Frame::EnableObs(int id) {
    auto iter = features_.find(id);
    if (iter != features_.end()) {
        iseff_[iter->first] = true;
    }
}

void Frame::DisableObs(int id) {
    auto iter = features_.find(id);
    if (iter != features_.end()) {
        iseff_[iter->first] = false;
    }
}

int Frame::GetEffObsNum() const {
    int cnt = 0;
    for (auto iter = iseff_.begin(); iter != iseff_.end(); ++iter) {
        if (iter->second) {
            cnt++;
        }
    }
    return cnt;
}

bool Frame::IsEffeObs(int id) const {
    auto iter = iseff_.find(id);
    if (iter != iseff_.end()) {
        return iter->second;
    } else {
        return false;
    }
}

std::vector<Feature> Frame::GetFeatures() const {
    std::vector<Feature> feats;
    for (auto iter = features_.begin(); iter != features_.end(); ++iter) {
        feats.push_back(iter->second);
    }
    return feats;
}

Feature Frame::GetFeature(int fid) const {
    auto iter = features_.find(fid);
    return iter->second;
}

} // namespace hityavie