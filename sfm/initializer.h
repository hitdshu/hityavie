#pragma once

#include <memory>
#include "sfm/frame.h"
#include "sfm/map.h"
#include "camera/camera_base.h"

namespace hityavie {

class Initializer {
public:
    typedef std::shared_ptr<Initializer> Ptr;

    Initializer(const BaseCamera::Ptr &cam, const Map::Ptr &map);
    ~Initializer() = default;

    bool Initialize(const Frame::Ptr &frm);

    Initializer(const Initializer &) = delete;
    Initializer &operator=(const Initializer &) = delete;

private:
    Frame::Ptr last_frm_;
    BaseCamera::Ptr cam_;
    Map::Ptr map_;
};

} // namespace hityavie