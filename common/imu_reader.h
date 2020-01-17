#pragma once

#include <map>
#include <string>
#include <Eigen/Dense>

namespace hityavie {

struct ImuData {
    double timestamp;
    Eigen::Vector3d ang_vel;
    Eigen::Vector3d lin_acc;
};

class ImuReader {
public:
    ImuReader() = default;
    ~ImuReader() = default;

    bool Init(const std::string &data_file_path, double min_timestamp = 0);
    std::map<double, ImuData> GetAllImuData() const;
    ImuData GetImu4Timestamp(double timestamp) const;
    std::vector<ImuData> GetImuDataBetweenImages(double timestamp1, double timestamp2) const;

    ImuReader(const ImuReader &) = delete;
    ImuReader &operator=(const ImuReader &) = delete;

private:
    std::map<double, ImuData> timestamp_imu_map_; 
};

} // namespace hityavie