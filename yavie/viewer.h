#pragma once

#include <thread>
#include <atomic>
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
    void Close();

protected:
    void Show();
    void DrawPoint();
    void DrawCamera();

private:
    std::thread vt_;
    std::atomic_bool is_running_;

    YavieMap::Ptr map_;
    BaseCamera::Ptr cam_;
};

} // namespace hityavie