#include "map/frame.h"

namespace hityavie {

int Frame::next_id_ = 0;

std::vector<int> Frame::Init(const std::vector<std::pair<Eigen::Vector2d, std::shared_ptr<Point>>> &obss) {
    id_ = next_id_++;
    std::vector<int> fids;
    int fid = 0;
    for (const auto &obs : obss) {
        fid_obs_map_[fid] = obs.first;
        fid_pt_map_[fid] = obs.second;
        fid++;
        fids.push_back(fid);
    }
    return fids;
}

void Frame::RemovePointObs(const std::shared_ptr<Point> &pt) {
    for (auto iter = fid_pt_map_.begin(); iter != fid_pt_map_.end(); ++iter) {
        if (iter->second == pt) {
            fid_pt_map_.erase(iter);
            return;
        }
    }
}

} // namespace hityavie