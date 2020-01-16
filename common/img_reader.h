#pragma once

#include <opencv2/opencv.hpp>
#include <map>

namespace hityavie {

class ImgReader {
public:
    ImgReader() = default;
    ~ImgReader() = default;

    bool Init(const std::string &file_path, const std::string &root_dir, double timeoffset = 0.0);
    std::vector<int> GetIds() const;
    double GetTimestamp4Id(int id) const;
    cv::Mat GetImg4Id(int id) const;

    ImgReader(const ImgReader &) = delete;
    ImgReader &operator=(const ImgReader &) = delete;

private:
    std::map<int, std::string> id_path_map_;
    std::map<int, double> id_timestamp_map_;
};

} // namespace hityavie