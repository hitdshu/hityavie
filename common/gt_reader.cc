#include <fstream>
#include <iostream>
#include "common/gt_reader.h"

namespace hityavie {

bool GtReader::Init(const std::string &data_file_path) {
    std::ifstream ifs(data_file_path);
    std::string line;
    while (std::getline(ifs, line)) {
        if (line[0] == '#') {
            continue;
        }
        int id;
        double timestamp;
        Eigen::Vector3d p;
        Eigen::Vector4d q;
        std::stringstream ss(line);
        ss >> id >> timestamp >> p[0] >> p[1] >> p[2] >> q[0] >> q[1] >> q[2] >> q[3];
        GtPose data;
        data.timestamp = timestamp;
        data.p = p;
        data.q = Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
        timestamp_pose_map_[data.timestamp] = data;
    }

    // std::ofstream out_pose("gt.txt");
    // out_pose.precision(20);
    // for (auto &iter : timestamp_pose_map_) {
    //     if (iter.first < 4910.5331293979079419 || iter.first > 4943.7399574319078965) {
    //         continue;
    //     }
    //     double timestamp = iter.first;
    //     Eigen::Vector3d p = iter.second.p;
    //     Eigen::Quaterniond q(iter.second.q);
    //     out_pose << timestamp << " " << p[0] << " " << p[1] << " " << p[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    // }
}

GtPose GtReader::GetGt4Time(double timestamp) const {
    auto iter = timestamp_pose_map_.upper_bound(timestamp);
    if (iter != timestamp_pose_map_.end()) {
        return iter->second;
    } else {
        GtPose data;
        data.timestamp = -1;
        return data;
    }
}

} // namespace hityavie