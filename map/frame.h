#pragma once

#include <map>
#include <Eigen/Dense>
#include "map/point.h"

namespace hityavie {

class Frame {
public:
    Frame() = default;
    ~Frame() = default;

    std::vector<int> Init(const std::vector<std::pair<Eigen::Vector2d, std::shared_ptr<Point>>> &obss);
    void RemovePointObs(const std::shared_ptr<Point> &pt);

    Frame(const Frame &) = delete;
    Frame &operator=(const Frame &) = delete;

private:
    static int next_id_;

    int id_;
    std::map<int, Eigen::Vector2d> fid_obs_map_;
    std::map<int, std::shared_ptr<Point>> fid_pt_map_;
};

} // namespace hityavie