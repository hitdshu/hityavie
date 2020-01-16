#include "map/sfm.h"

namespace hityavie {

Sfm::Sfm(const Map::Ptr &map, const BaseCamera::Ptr &cam) {
    map_ = map;
    cam_ = cam;
    initer_.reset(new Initializer(cam, map));
    state_ = kSfmEmpty;
}

void Sfm::PushFrame(Frame::Ptr &frm) {
    if (kSfmEmpty == state_) {
        bool flag = initer_->Initialize(frm);
        if (flag) {
            state_ = kSfmInited;
        }
        return;
    } else if (kSfmInited == state_) {

    }
}

} // namespace hityavie