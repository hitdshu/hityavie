#pragma once

#include <memory>
#include <map>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "proto/param.pb.h"
#include "sfm/sfm.h"
#include "camera/camera_base.h"
#include "yavie/rotation_calib.h"
#include "imu/preintegrator.h"
#include "yavie/yavie_map.h"

namespace hityavie {

enum YavieState {
    kYavieInit = 0,
    kYavieSfm,
    kYavieCalib,
    kYavieVialign,
    kYavieTracking,
    kYavieLost, 
    kYavieStateNum
};

struct YavieImuData {
    double timestamp;
    Eigen::Vector3d ang_vel;
    Eigen::Vector3d lin_acc;
};

class YavieEstimator {
public:
    typedef std::shared_ptr<YavieEstimator> Ptr;
    typedef std::pair<Frame::Ptr, double> FrameWithTime;

    YavieEstimator(const YavieParameter &param);
    ~YavieEstimator() = default;

    void ProcessImg(double timestamp, cv::Mat &img);
    void ProcessImu(double timestamp, const Eigen::Vector3d &lin_acc, const Eigen::Vector3d &ang_vel);

    YavieState GetState() const {
        return state_;
    }
    YavieMap::Ptr GetMap() const {
        return map_;
    }
    BaseCamera::Ptr GetCamera() const {
        return cam_;
    }

    YavieEstimator(const YavieEstimator &) = delete;
    YavieEstimator &operator=(const YavieEstimator &) = delete;

protected:
    void YavieInit(double timestamp, cv::Mat &img);
    void YavieSfm(double timestamp, cv::Mat &img);
    void VisualInertialAlign();
    void CalcBg();
    void LinearAlignment();
    Eigen::VectorXd RefineGravity(const Eigen::VectorXd &init_x);
    void VisualInertialInit(const Eigen::VectorXd &x);

private:
    std::vector<YavieImuData> GetImuDataBetweenImages(double timestamp1, double timestamp2) const;
    YavieImuData InterpolateImuData(const YavieImuData &d1, const YavieImuData &d2, double timestamp) const;
    YavieImuData GetImuData(double timestamp) const;

    YavieParameter param_;
    YavieState state_;

    std::map<double, YavieImuData> timestamp_imu_map_;
    double last_img_timestamp_;
    YavieImuData last_imu_data_;

    BaseCamera::Ptr cam_;
    BaseTracker::Ptr tracker_;
    YavieMap::Ptr map_;

    Eigen::Vector3d gravity_;

    Sfm::Ptr sfm_;
    RotationCalib::Ptr rot_calibr_;
    std::vector<FrameWithTime> init_frms_;
    std::vector<Preintegrator::Ptr> init_ints_;
};

} // namespace hityavie