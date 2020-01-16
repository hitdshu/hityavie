#pragma once

#include <memory>
#include "map/map.h"
#include "map/initializer.h"
#include "camera/camera_base.h"

namespace hityavie {

enum SfmState {
    kSfmEmpty = 0,
    kSfmInited = 1,
    kSfmDone = 2
};

class Sfm {
public:
    typedef std::shared_ptr<Sfm> Ptr;

    Sfm(const Map::Ptr &map, const BaseCamera::Ptr &cam);
    ~Sfm() = default;

    void PushFrame(Frame::Ptr &frm);

    SfmState GetState() const {
        return state_;
    }

    Sfm(const Sfm &) = delete;
    Sfm &operator=(const Sfm &) = delete;

protected:
    void SolvePnp(const Frame::Ptr &last_frm, const Frame::Ptr &cur_frm);

private:
    Map::Ptr map_;
    BaseCamera::Ptr cam_;
    Initializer::Ptr initer_;
    SfmState state_;
};

} // namespace hityavie