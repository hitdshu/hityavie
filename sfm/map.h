#pragma once

#include <mutex>
#include "sfm/point.h"
#include "sfm/frame.h"

namespace hityavie {

class Map {
public:
    typedef std::shared_ptr<Map> Ptr;

    Map() = default;
    ~Map() = default;

    void AddPoint(const Point::Ptr &pt);
    void AddFrame(const Frame::Ptr &frm);
    void Scale(double scale);

    std::vector<Point::Ptr> GetAllPoints() const;
    std::vector<Frame::Ptr> GetAllFrames() const;
    Point::Ptr GetPoint(int pid) const;
    Frame::Ptr GetFrame(int fid) const;
    Frame::Ptr GetLastFrame() const;
    int GetFrameCnt() const;
    int GetPointCnt() const;
    bool HasPoint(int pid) const;
    bool HasFrame(int fid) const;

    Map(const Map &) = delete;
    Map &operator=(const Map &) = delete;

private:
    std::map<int, Point::Ptr> id_pt_map_;
    std::map<int, Frame::Ptr> id_frm_map_;
    std::mutex mutex_;
};

} // namespace hityavie