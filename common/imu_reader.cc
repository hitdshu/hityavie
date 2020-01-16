#include <fstream>
#include "common/imu_reader.h"

namespace hityavie {

bool ImuReader::Init(const std::string &data_file_path, double min_timestamp) {
    std::ifstream ifs(data_file_path);
    std::string line;
    while (std::getline(ifs, line)) {
        if (line[0] == '#') {
            continue;
        }
        int id;
        ImuData data;
        std::stringstream ss(line);
        ss >> id >> data.timestamp >> data.ang_vel[0] >> data.ang_vel[1] >> data.ang_vel[2] >> data.lin_acc[0] >> data.lin_acc[1] >> data.lin_acc[2];
        if (data.timestamp > min_timestamp)
            timestamp_imu_map_[data.timestamp] = data;
    }
}

std::map<double, ImuData> ImuReader::GetAllImuData() const {
    return timestamp_imu_map_;
}

ImuData ImuReader::GetImu4Timestamp(double timestamp) const {
    auto iter = timestamp_imu_map_.find(timestamp);
    if (iter != timestamp_imu_map_.end()) {
        return iter->second;
    } else {
        ImuData data;
        data.timestamp = -1;
        return data;
    }
}

} // namespace hityavie