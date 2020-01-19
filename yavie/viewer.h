#pragma once

#include <thread>
#include <pangolin/pangolin.h>
#include "yavie/yavie_map.h"
#include "camera/camera_base.h"

namespace hityavie {

class Viewer {
public:
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer() = default;
    ~Viewer() = default;

    void InitViewer(const YavieMap::Ptr &map, const BaseCamera::Ptr cam);
    void Show();

protected:
    void DrawPoint();
    void DrawCamera();

private:
    YavieMap::Ptr map_;
    BaseCamera::Ptr cam_;
    std::thread vt_;
};

} // namespace hityavie