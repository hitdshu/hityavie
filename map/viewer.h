#pragma once

#include <thread>
#include <pangolin/pangolin.h>
#include "map/map.h"

namespace hityavie {

class Viewer {
public:
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer() = default;
    ~Viewer() = default;

    void InitViewer(const Map::Ptr &map);
    void Show();

protected:
    void DrawPoint();
    void DrawCamera();

private:
    Map::Ptr map_;
    std::thread vt_;
};

} // namespace hityavie