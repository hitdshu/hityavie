#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <memory>
#include "common/register.h"
#include "proto/param.pb.h"

namespace hityavie {

class BaseCamera {
public:
    typedef std::shared_ptr<BaseCamera> Ptr;

    BaseCamera() {
        id_ = next_id_++;
    }
    virtual ~BaseCamera() = default;

    virtual bool Init(const CameraParam &param) = 0;
    virtual cv::Mat GetMatK() const = 0;
    virtual bool PreProcess(cv::Mat &img) const = 0;
    virtual Eigen::Vector2d UndistortPt(const Eigen::Vector2d &pt) const = 0;
    virtual std::vector<Eigen::Vector2d> UndistortPts(const std::vector<Eigen::Vector2d> &pts) const = 0;
    virtual Eigen::Vector3d Pt2Ray(const Eigen::Vector2d &pt) const = 0;
    virtual Eigen::Vector2d Project(const Eigen::Vector3d &pc) const = 0;
    virtual Eigen::Matrix<double, 2, 3> Jacobian(const Eigen::Vector3d &pc) const = 0;
    virtual std::string Name() const = 0;
    virtual Eigen::Matrix4d GetTfic() const = 0;
    virtual Eigen::Matrix3d GetRic() const = 0;
    virtual Eigen::Vector3d GetTic() const = 0;
    virtual int GetId() const final {
        return id_;
    }

    BaseCamera(const BaseCamera &) = delete;
    BaseCamera &operator=(const BaseCamera &) = delete;

private:
    static int next_id_;
    int id_;
};

HITYAVIE_REGISTER_REGISTERER(BaseCamera);
#define HITYAVIE_REGISTER_CAMERA(name) \
    HITYAVIE_REGISTER_CLASS(BaseCamera, name)

} // namespace hityavie