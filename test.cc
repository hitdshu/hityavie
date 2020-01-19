#include <iostream>
#include "common/parser.h"
#include "common/imu_reader.h"
#include "common/img_reader.h"
#include "common/timer.h"
#include "yavie/rotation_calib.h"
#include "yavie/yavie_estimator.h"
#include "yavie/viewer.h"

using namespace hityavie;

int main(int argc, char **argv) {
    const std::string cfg_folder(argv[1]);
    const std::string data_folder(argv[2]);
    const std::string imu_file(data_folder + "/imu.txt");
    const std::string img_root_dir(data_folder);
    const std::string img_file(data_folder + "/images.txt");
    const int start_idx = 928;
    const int end_idx = 1820;
    ImuReader imu_reader;
    imu_reader.Init(imu_file);
    const double img_timeoff = -0.0166845720919;
    ImgReader img_reader;
    img_reader.Init(img_file, img_root_dir, img_timeoff);
    YavieParameter param;
    Parser::ParsePrototxt<YavieParameter>(cfg_folder + "/param.prototxt", param);
    YavieEstimator::Ptr estimator(new YavieEstimator(param));
    Viewer::Ptr viewer(new Viewer());
    viewer->InitViewer(estimator->GetMap(), estimator->GetCamera());
    std::map<double, ImuData> all_imu_data = imu_reader.GetAllImuData();
    for (auto iter = all_imu_data.begin(); iter != all_imu_data.end(); ++iter) {
        estimator->ProcessImu(iter->second.timestamp, iter->second.lin_acc, iter->second.ang_vel);
    }

    for (int img_idx = start_idx; img_idx <= end_idx; ++img_idx) {
        std::cout << "Img idx " << img_idx << std::endl;
        double img_timestamp = img_reader.GetTimestamp4Id(img_idx);
        cv::Mat img = img_reader.GetImg4Id(img_idx);
        estimator->ProcessImg(img_timestamp, img);
        std::cout << "Yavie estimator state: " << estimator->GetState() << std::endl;
        if (estimator->GetState() == kYavieTracking) {
            cv::imshow("img", img);
            cv::waitKey(0);
        }
    }


    return 0;
}