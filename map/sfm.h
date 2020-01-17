#pragma once

#include <memory>
#include <set>
#include "map/map.h"
#include "map/initializer.h"
#include "camera/camera_base.h"
#include "proto/param.pb.h"

namespace hityavie {

enum SfmState {
    kSfmEmpty = 0,
    kSfmInited = 1,
    kSfmDone = 2
};

class Sfm {
public:
    typedef std::shared_ptr<Sfm> Ptr;

    Sfm(const Map::Ptr &map, const BaseCamera::Ptr &cam, const SfmParam &sp);
    ~Sfm() = default;

    void PushFrame(Frame::Ptr &frm);

    Eigen::Matrix4d GetRelativePose() const;
    SfmState GetState() const {
        return state_;
    }

    Sfm(const Sfm &) = delete;
    Sfm &operator=(const Sfm &) = delete;

protected:
    void SolvePnp(const Frame::Ptr &last_frm, const Frame::Ptr &cur_frm);
    void AddKeyFrame(const Frame::Ptr &last_frm, const Frame::Ptr &cur_frm);
    void LocalOptimize();
    void GlobalOptimize();

private:
    Map::Ptr map_;
    BaseCamera::Ptr cam_;
    SfmParam sp_;
    Eigen::Matrix4d tcpcc_;
    Eigen::Matrix4d twcp_;
    Initializer::Ptr initer_;
    SfmState state_;
};

} // namespace hityavie