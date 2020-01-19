#include "yavie/yavie_point.h"

namespace hityavie {

YaviePoint::YaviePoint(int id, const Eigen::Vector3d &p) {
    id_ = id;
    position_ = p;
}

YaviePoint::~YaviePoint() {
    for (auto iter = frms_.begin(); iter != frms_.end(); ++iter) {
        (*iter)->DisableObs(id_);
    }
}

void YaviePoint::AddObs(const YavieFrame::Ptr &frm) {
    frms_.insert(frm);
}

void YaviePoint::RmObs(const YavieFrame::Ptr &frm) {
    auto iter = frms_.find(frm);
    if (iter != frms_.end()) {
        frms_.erase(iter);
    }
}

void YaviePoint::SetPosition(const Eigen::Vector3d &p) {
    position_ = p;
}

std::vector<YavieFrame::Ptr> YaviePoint::GetAllObs() const {
    std::vector<YavieFrame::Ptr> frms;
    for (auto iter = frms_.begin(); iter != frms_.end(); ++iter) {
        frms.push_back(*iter);
    }
    return frms;
}

Eigen::Vector3d YaviePoint::GetPosition() const {
    return position_;
}

int YaviePoint::GetObsNum() const {
    return frms_.size();
}

int YaviePoint::GetId() const {
    return id_;
}

} // namespace hityavie