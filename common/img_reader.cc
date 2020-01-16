#include "common/img_reader.h"

namespace hityavie {

bool ImgReader::Init(const std::string &file_path, const std::string &root_dir, double timeoffset) {
    std::ifstream ifs(file_path);
    std::string line;
    while (std::getline(ifs, line)) {
        if (line[0] == '#') {
            continue;
        }
        int id;
        double timestamp;
        std::string path;
        std::stringstream ss(line);
        ss >> id >> timestamp >> path;
        id_timestamp_map_[id] = timestamp + timeoffset;
        id_path_map_[id] = root_dir + "/" + path;
    }
}

std::vector<int> ImgReader::GetIds() const {
    std::vector<int> ids;
    for (auto iter = id_timestamp_map_.begin(); iter != id_timestamp_map_.end(); ++iter) {
        ids.push_back(iter->first);
    }
    return ids;
}

double ImgReader::GetTimestamp4Id(int id) const {
    auto iter = id_timestamp_map_.find(id);
    if (iter != id_timestamp_map_.end()) {
        return iter->second;
    } else {
        return -1;
    }
}

cv::Mat ImgReader::GetImg4Id(int id) const {
    auto iter = id_path_map_.find(id);
    if (iter != id_path_map_.end()) {
        cv::Mat img = cv::imread(iter->second, cv::IMREAD_GRAYSCALE);
        return img;
    } else {
        return cv::Mat();
    }
}

} // namespace hityavie