#include <iostream>
#include "common/parser.h"
#include "common/imu_reader.h"
#include "common/img_reader.h"
#include "common/gt_reader.h"
#include "common/timer.h"
#include "yavie/rotation_calib.h"
#include "yavie/yavie_estimator.h"
#include "yavie/viewer.h"

using namespace hityavie;

int main(int argc, char **argv) {
    const std::string cfg_folder(argv[1]);
    const std::string data_folder(argv[2]);
    const std::string imu_file(data_folder + "/imu.txt");
    const std::string gt_file(data_folder + "/groundtruth.txt");
    const std::string img_root_dir(data_folder);
    const std::string img_file(data_folder + "/images.txt");
    const int start_idx = 928;
    const int end_idx = 1820;
    ImuReader imu_reader;
    imu_reader.Init(imu_file);
    const double img_timeoff = -0.0166845720919;
    ImgReader img_reader;
    img_reader.Init(img_file, img_root_dir, img_timeoff);
    GtReader::Ptr gt_reader(new GtReader());
    gt_reader->Init(gt_file);
    YavieParameter param;
    Parser::ParsePrototxt<YavieParameter>(cfg_folder + "/param.prototxt", param);
    YavieEstimator::Ptr estimator(new YavieEstimator(param));
    Viewer::Ptr viewer(new Viewer());
    viewer->InitViewer(estimator->GetMap(), estimator->GetCamera(), gt_reader);
    std::map<double, ImuData> all_imu_data = imu_reader.GetAllImuData();
    for (auto iter = all_imu_data.begin(); iter != all_imu_data.end(); ++iter) {
        estimator->ProcessImu(iter->second.timestamp, iter->second.lin_acc, iter->second.ang_vel);
    }
    Timer timer;

    for (int img_idx = start_idx; img_idx <= end_idx; ++img_idx) {
        std::cout << "Img idx " << img_idx << std::endl;
        double img_timestamp = img_reader.GetTimestamp4Id(img_idx);
        cv::Mat img = img_reader.GetImg4Id(img_idx);
        timer.Tic();
        estimator->ProcessImg(img_timestamp, img);
        timer.Toc("one frame estimation");
        std::cout << "Yavie estimator state: " << estimator->GetState() << std::endl;
    }

    std::ofstream out_pose("out_pose.txt");
    out_pose.precision(20);
    auto all_frms = estimator->GetMap()->GetAllFrames();
    for (auto &frm : all_frms) {
        Eigen::Matrix4d twb = frm->GetPose();
        Eigen::Matrix3d rot = twb.block(0, 0, 3, 3);
        Eigen::Vector3d p = twb.block(0, 3, 3, 1);
        Eigen::Quaterniond q(rot);
        out_pose << frm->GetTimestamp() << " " << p[0] << " " << p[1] << " " << p[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }

    viewer->Close();
    return 0;
}
