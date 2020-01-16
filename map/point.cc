#include "map/point.h"

namespace hityavie {

Point::Point(int id, const Eigen::Vector3d &p) {
    id_ = id;
    position_ = p;
}

void Point::AddObs(const Frame::Ptr &frm) {
    frms_.insert(frm);
}

void Point::RmObs(const Frame::Ptr &frm) {
    auto iter = frms_.find(frm);
    if (iter != frms_.end()) {
        frms_.erase(iter);
    }
}

void Point::SetPosition(const Eigen::Vector3d &p) {
    position_ = p;
}

std::vector<Frame::Ptr> Point::GetAllObs() const {
    std::vector<Frame::Ptr> frms;
    for (auto iter = frms_.begin(); iter != frms_.end(); ++iter) {
        frms.push_back(*iter);
    }
    return frms;
}

Eigen::Vector3d Point::GetPosition() const {
    return position_;
}

int Point::GetObsNum() const {
    return frms_.size();
}

int Point::GetId() const {
    return id_;
}

} // namespace hityavie