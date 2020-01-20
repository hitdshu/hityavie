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
    void SetCurFrame(const YavieFrame::Ptr &cf);

    std::vector<YaviePoint::Ptr> GetAllPoints();
    std::vector<YavieFrame::Ptr> GetAllFrames();
    YaviePoint::Ptr GetPoint(int pid);
    YavieFrame::Ptr GetFrame(int fid);
    YavieFrame::Ptr GetLastFrame();
    YavieFrame::Ptr GetCurFrame();
    int GetFrameCnt();
    int GetPointCnt();
    bool HasPoint(int pid);
    bool HasFrame(int fid);

    YavieMap(const YavieMap &) = delete;
    YavieMap &operator=(const YavieMap &) = delete;

private:
    std::map<int, YaviePoint::Ptr> id_pt_map_;
    std::map<int, YavieFrame::Ptr> id_frm_map_;
    YavieFrame::Ptr cf_;
    std::mutex mutex_;
};

} // namespace hityavie