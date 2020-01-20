#include "yavie/yavie_map.h"

namespace hityavie {

void YavieMap::AddPoint(const YaviePoint::Ptr &pt) {
    std::unique_lock<std::mutex> lock(mutex_);
    id_pt_map_[pt->GetId()] = pt;
}

void YavieMap::AddFrame(const YavieFrame::Ptr &frm) {
    std::unique_lock<std::mutex> lock(mutex_);
    id_frm_map_[frm->GetId()] = frm;
}

void YavieMap::RmPoint(int id) {
    std::unique_lock<std::mutex> lock(mutex_);
    auto iter = id_pt_map_.find(id);
    if (iter != id_pt_map_.end()) {
        id_pt_map_.erase(iter);
    }
}

void YavieMap::RmFrame(int id) {
    std::unique_lock<std::mutex> lock(mutex_);
    auto iter = id_frm_map_.find(id);
    if (iter != id_frm_map_.end()) {
        id_frm_map_.erase(iter);
    }
}

void YavieMap::SetCurFrame(const YavieFrame::Ptr &cf) {
    std::unique_lock<std::mutex> lock(mutex_);
    cf_ = cf;
}

std::vector<YaviePoint::Ptr> YavieMap::GetAllPoints() {
    std::unique_lock<std::mutex> lock(mutex_);
    std::vector<YaviePoint::Ptr> all_pts;
    for (auto iter = id_pt_map_.begin(); iter != id_pt_map_.end(); ++iter) {
        all_pts.push_back(iter->second);
    }
    return all_pts;
}

std::vector<YavieFrame::Ptr> YavieMap::GetAllFrames() {
    std::unique_lock<std::mutex> lock(mutex_);
    std::vector<YavieFrame::Ptr> all_frms;
    for (auto iter = id_frm_map_.begin(); iter != id_frm_map_.end(); ++iter) {
        all_frms.push_back(iter->second);
    }
    return all_frms;
}

YaviePoint::Ptr YavieMap::GetPoint(int pid) {
    std::unique_lock<std::mutex> lock(mutex_);
    auto iter = id_pt_map_.find(pid);
    if (iter != id_pt_map_.end()) {
        return iter->second;
    } else {
        return YaviePoint::Ptr();
    }
}

YavieFrame::Ptr YavieMap::GetFrame(int fid) {
    std::unique_lock<std::mutex> lock(mutex_);
    auto iter = id_frm_map_.find(fid);
    if (iter != id_frm_map_.end()) {
        return iter->second;
    } else {
        return YavieFrame::Ptr();
    }
}

YavieFrame::Ptr YavieMap::GetLastFrame() {
    std::unique_lock<std::mutex> lock(mutex_);
    auto iter = id_frm_map_.end();
    return (--iter)->second;
}

YavieFrame::Ptr YavieMap::GetCurFrame() {
    return cf_;
}

int YavieMap::GetFrameCnt() {
    std::unique_lock<std::mutex> lock(mutex_);
    return id_frm_map_.size();
}

int YavieMap::GetPointCnt() {
    std::unique_lock<std::mutex> lock(mutex_);
    return id_pt_map_.size();
}

bool YavieMap::HasPoint(int pid) {
    std::unique_lock<std::mutex> lock(mutex_);
    auto iter = id_pt_map_.find(pid);
    if (iter != id_pt_map_.end()) {
        return true;
    } else {
        return false;
    }
}

bool YavieMap::HasFrame(int fid) {
    std::unique_lock<std::mutex> lock(mutex_);
    auto iter = id_frm_map_.find(fid);
    if (iter != id_frm_map_.end()) {
        return true;
    } else {
        return false;
    }
}

} // namespace hityavie