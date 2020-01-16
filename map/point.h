#pragma once

#include <memory>
#include <map>
#include <vector>

namespace hityavie {

class Frame;

class Point {
public:
    Point(int id);
    ~Point() = default;

    void AddObservation(const std::shared_ptr<Frame> &frm, int fid);
    void RemoveObservation(const std::shared_ptr<Frame> &frm);
    void SetInvDepth(double inv_depth);

    std::map<std::shared_ptr<Frame>, int> GetAllObs() const;
    std::vector<std::shared_ptr<Frame>> GetAllFrms() const;
    int GetFid4Frm(const std::shared_ptr<Frame> &frm) const;
    int GetId() const;
    int GetObsNum() const;
    double GetInvDepth() const;
    
    Point(const Point &) = delete;
    Point &operator=(const Point &) = delete;

private:
    int id_;
    double inv_depth_;
    std::shared_ptr<Frame> anchor_frm_;
    std::map<std::shared_ptr<Frame>, int> frm_fid_map_;
};

} // namespace hityavie