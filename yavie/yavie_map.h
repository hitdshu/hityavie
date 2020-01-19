#pragma once

#include <mutex>
#include <map>
#include "yavie/yavie_point.h"
#include "yavie/yavie_frame.h"

namespace hityavie {

class YavieMap {
public:
    typedef std::shared_ptr<YavieMap> Ptr;

    YavieMap() = default;
    ~YavieMap() = default;

    void AddPoint(const YaviePoint::Ptr &pt);
    void AddFrame(const YavieFrame::Ptr &frm);
    void RmPoint(int id);
    void RmFrame(int id);

    std::vector<YaviePoint::Ptr> GetAllPoints() const;
    std::vector<YavieFrame::Ptr> GetAllFrames() const;
    YaviePoint::Ptr GetPoint(int pid) const;
    YavieFrame::Ptr GetFrame(int fid) const;
    YavieFrame::Ptr GetLastFrame() const;
    int GetFrameCnt() const;
    int GetPointCnt() const;
    bool HasPoint(int pid) const;
    bool HasFrame(int fid) const;

    YavieMap(const YavieMap &) = delete;
    YavieMap &operator=(const YavieMap &) = delete;

private:
    std::map<int, YaviePoint::Ptr> id_pt_map_;
    std::map<int, YavieFrame::Ptr> id_frm_map_;
    std::mutex mutex_;
};

} // namespace hityavie