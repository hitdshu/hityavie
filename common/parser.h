#pragma once

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

namespace hityavie {

class Parser {
public:
    template <typename ValueType>
    static bool ParsePrototxt(const std::string &file_path, ValueType &val);
};

template <typename ValueType>
bool Parser::ParsePrototxt(const std::string &file_path, ValueType &val) {
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    int fd = open(file_path.c_str(), O_RDONLY);
    google::protobuf::io::FileInputStream* input = new google::protobuf::io::FileInputStream(fd);
    bool success = google::protobuf::TextFormat::Parse(input, &val);
    if (!success) {
        close(fd);
        delete input;
        return false;
    } else {
        close(fd);
        delete input;
        return true;
    }
}

} // namespace hityavie