#pragma once

#include <map>
#include <string>
#include <memory>
#include <Eigen/Dense>

namespace hityavie {

struct GtPose {
    double timestamp;
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
};

class GtReader {
public:
    typedef std::shared_ptr<GtReader> Ptr;

    GtReader() = default;
    ~GtReader() = default;

    bool Init(const std::string &data_file_path);
    GtPose GetGt4Time(double timestamp) const;

    GtReader(const GtReader &) = delete;
    GtReader &operator=(const GtReader &) = delete;

private:
    std::map<double, GtPose> timestamp_pose_map_; 
};

} // namespace hityavie