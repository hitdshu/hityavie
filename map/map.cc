#include "map/map.h"

namespace hityavie {

void Map::AddPoint(const Point::Ptr &pt) {
    std::unique_lock<std::mutex> lock(mutex_);
    id_pt_map_[pt->GetId()] = pt;
}

void Map::AddFrame(const Frame::Ptr &frm) {
    std::unique_lock<std::mutex> lock(mutex_);
    id_frm_map_[frm->GetId()] = frm;
}

std::vector<Point::Ptr> Map::GetAllPoints() const {
    std::vector<Point::Ptr> all_pts;
    for (auto iter = id_pt_map_.begin(); iter != id_pt_map_.end(); ++iter) {
        all_pts.push_back(iter->second);
    }
    return all_pts;
}

std::vector<Frame::Ptr> Map::GetAllFrames() const {
    std::vector<Frame::Ptr> all_frms;
    for (auto iter = id_frm_map_.begin(); iter != id_frm_map_.end(); ++iter) {
        all_frms.push_back(iter->second);
    }
    return all_frms;
}

Point::Ptr Map::GetPoint(int pid) const {
    auto iter = id_pt_map_.find(pid);
    if (iter != id_pt_map_.end()) {
        return iter->second;
    } else {
        return Point::Ptr();
    }
}

Frame::Ptr Map::GetFrame(int fid) const {
    auto iter = id_frm_map_.find(fid);
    if (iter != id_frm_map_.end()) {
        return iter->second;
    } else {
        return Frame::Ptr();
    }
}

Frame::Ptr Map::GetLastFrame() const {
    auto iter = id_frm_map_.end();
    return (--iter)->second;
}

int Map::GetFrameCnt() const {
    return id_frm_map_.size();
}

int Map::GetPointCnt() const {
    return id_pt_map_.size();
}

bool Map::HasPoint(int pid) const {
    auto iter = id_pt_map_.find(pid);
    if (iter != id_pt_map_.end()) {
        return true;
    } else {
        return false;
    }
}

bool Map::HasFrame(int fid) const {
    auto iter = id_frm_map_.find(fid);
    if (iter != id_frm_map_.end()) {
        return true;
    } else {
        return false;
    }
}

} // namespace hityavie