#pragma once

#include <memory>
#include <map>
#include <vector>
#include <set>
#include <Eigen/Dense>
#include "yavie/yavie_frame.h"

namespace hityavie {

class YaviePoint {
public:
    typedef std::shared_ptr<YaviePoint> Ptr;

    YaviePoint(int id, const Eigen::Vector3d &position);
    ~YaviePoint();

    void AddObs(const YavieFrame::Ptr &frm);
    void RmObs(const YavieFrame::Ptr &frm);
    void SetPosition(const Eigen::Vector3d &p);

    std::vector<YavieFrame::Ptr> GetAllObs() const;
    Eigen::Vector3d GetPosition() const;
    int GetObsNum() const;
    int GetId() const;
    
    YaviePoint(const YaviePoint &) = delete;
    YaviePoint &operator=(const YaviePoint &) = delete;

private:
    int id_;
    Eigen::Vector3d position_;
    std::set<YavieFrame::Ptr> frms_;
};

} // namespace hityavie