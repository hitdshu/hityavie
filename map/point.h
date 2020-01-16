#pragma once

#include <memory>
#include <map>
#include <vector>
#include <set>
#include <Eigen/Dense>
#include "map/frame.h"

namespace hityavie {

class Point {
public:
    typedef std::shared_ptr<Point> Ptr;

    Point(int id, const Eigen::Vector3d &position);
    ~Point() = default;

    void AddObs(const Frame::Ptr &frm);
    void RmObs(const Frame::Ptr &frm);
    void SetPosition(const Eigen::Vector3d &p);

    std::vector<Frame::Ptr> GetAllObs() const;
    Eigen::Vector3d GetPosition() const;
    int GetObsNum() const;
    int GetId() const;
    
    Point(const Point &) = delete;
    Point &operator=(const Point &) = delete;

private:
    int id_;
    Eigen::Vector3d position_;
    std::set<Frame::Ptr> frms_;
};

} // namespace hityavie