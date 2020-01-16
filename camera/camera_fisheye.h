#pragma once

#include "camera/camera_base.h"

namespace hityavie {

class CameraFisheye : public BaseCamera {
public:
    virtual bool Init(const CameraParam &param) override;
    virtual cv::Mat GetMatK() const override;
    virtual bool PreProcess(cv::Mat &img) const override;
    virtual Eigen::Vector2d UndistortPt(const Eigen::Vector2d &pt) const override;
    virtual std::vector<Eigen::Vector2d> UndistortPts(const std::vector<Eigen::Vector2d> &pts) const override;
    virtual Eigen::Vector3d Pt2Ray(const Eigen::Vector2d &pt) const override;
    virtual Eigen::Vector2d Project(const Eigen::Vector3d &pc) const override;
    virtual Eigen::Matrix<double, 2, 3> Jacobian(const Eigen::Vector3d &pc) const override;
    virtual std::string Name() const override;
    virtual Eigen::Matrix4d GetTfic() const override;
    virtual Eigen::Matrix3d GetRic() const override;
    virtual Eigen::Vector3d GetTic() const override;

private:
    FisheyeCameraParam param_;
    Eigen::Matrix4d tic_;
    cv::Mat k_;
    cv::Mat d_;
    cv::Mat mapx_;
    cv::Mat mapy_;
};

} // namespace hityavie