#include "map/point.h"
#include "map/frame.h"

namespace hityavie {

Point::Point(int id) {
    id_ = id;
    anchor_frm_ = nullptr;
    inv_depth_ = -1;
}

void Point::AddObservation(const std::shared_ptr<Frame> &frm, int fid) {
    if (!anchor_frm_) {
        anchor_frm_ = frm;
    }
    frm_fid_map_[frm] = fid;
}

void Point::RemoveObservation(const std::shared_ptr<Frame> &frm) {
    auto iter = frm_fid_map_.find(frm);
    if (iter != frm_fid_map_.end()) {
        frm_fid_map_.erase(iter);
    }
}

void Point::SetInvDepth(double inv_depth) {
    inv_depth_ = inv_depth;
}

std::map<std::shared_ptr<Frame>, int> Point::GetAllObs() const {
    return frm_fid_map_;
}

std::vector<std::shared_ptr<Frame>> Point::GetAllFrms() const {
    std::vector<std::shared_ptr<Frame>> all_frms;
    for (auto iter = frm_fid_map_.begin(); iter != frm_fid_map_.end(); ++iter) {
        all_frms.push_back(iter->first);
    }
    return all_frms;
}

int Point::GetFid4Frm(const std::shared_ptr<Frame> &frm) const {
    auto iter = frm_fid_map_.find(frm);
    if (iter != frm_fid_map_.end()) {
        return iter->second;
    } else {
        return -1;
    }
}

int Point::GetId() const {
    return id_;
}

int Point::GetObsNum() const {
    return frm_fid_map_.size();
}

double Point::GetInvDepth() const {
    return inv_depth_;
}

} // namespace hityavie