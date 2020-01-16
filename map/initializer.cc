#include "map/initializer.h"
#include "common/adapter.h"
#include "common/geometry_utility.h"

namespace hityavie {

Initializer::Initializer(const BaseCamera::Ptr &cam, const Map::Ptr &map) {
    cam_ = cam;
    map_ = map;
}

bool Initializer::Initialize(const Frame::Ptr &frm) {
    if (!last_frm_) {
        last_frm_ = frm;
        return false;
    }
    std::vector<int> common_feat_id;
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;
    double total_of = 0;
    std::vector<Feature> feats_last = last_frm_->GetFeatures();
    std::vector<Feature> feats_cur = frm->GetFeatures();
    for (auto &feat1 : feats_last) {
        for (auto &feat2 : feats_cur) {
            if (feat1.id == feat2.id) {
                common_feat_id.push_back(feat1.id);
                pts1.push_back(Adapter::Vec2Pt(feat1.pt_und));
                pts2.push_back(Adapter::Vec2Pt(feat2.pt_und));
                total_of = (feat1.pt_ori - feat2.pt_und).norm();
            }
        }
    }
    double avg_of = total_of / common_feat_id.size();
    if (avg_of < 0.05) {
        last_frm_ = frm;
        return false;
    }
    cv::Mat k = cam_->GetMatK();
    std::vector<uchar> status;
    cv::Mat e = cv::findEssentialMat(pts1, pts2, k, cv::RANSAC, 0.99, 1.0, status);
    cv::Mat r1;
    cv::Mat r2;
    cv::Mat t;
    cv::decomposeEssentialMat(e, r1, r2, t);
    cv::Mat r;
    if (r1.at<double>(0, 0) > r2.at<double>(0, 0)) {
        r = r1;
    } else {
        r = r2;
    }
    std::cout << r.type() << std::endl;
    Eigen::Matrix3d r21;
    Eigen::Vector3d t21;
    r21 << r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2), 
        r.at<double>(1, 0), r.at<double>(1, 1), r.at<double>(1, 2), 
        r.at<double>(2, 0), r.at<double>(2, 1), r.at<double>(2, 2);
    t21 << t.at<double>(0), t.at<double>(1), t.at<double>(2);
    Eigen::Matrix4d tc1w = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d tc2w_1 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d tc2w_2 = Eigen::Matrix4d::Identity();
    tc2w_1.block(0, 0, 3, 3) = r21;
    tc2w_1.block(0, 3, 3, 1) = t21;
    tc2w_2.block(0, 0, 3, 3) = r21;
    tc2w_2.block(0, 3, 3, 1) = -t21;
    int total_np = 0;
    int np1 = 0;
    int np2 = 0;
    std::vector<Eigen::Vector3d> pts_w1;
    std::vector<Eigen::Vector3d> pts_w2;
    for (size_t idx = 0; idx < common_feat_id.size(); ++idx) {
        Eigen::Vector2d p1 = Adapter::Pt2Vec(pts1[idx]);
        Eigen::Vector2d p2 = Adapter::Pt2Vec(pts2[idx]);
        Eigen::Vector3d pw_1 = GeometryUtility::Triangulate(tc1w, cam_->Pt2Ray(p1), tc2w_1, cam_->Pt2Ray(p2));
        Eigen::Vector3d pw_2 = GeometryUtility::Triangulate(tc1w, cam_->Pt2Ray(p1), tc2w_2, cam_->Pt2Ray(p2));
        pts_w1.push_back(pw_1);
        pts_w2.push_back(pw_2);
        if (pw_1[2] > 0) {
            np1++;
        }
        if (pw_2[2] > 0) {
            np2++;
        }
        total_np++;
    }
    Eigen::Matrix4d tc2w;
    std::vector<Eigen::Vector3d> ptsw;
    if (np2 > np1) {
        tc2w = tc2w_2;
        ptsw = pts_w2;
    } else {
        tc2w = tc2w_1;
        ptsw = pts_w1;
    }
    last_frm_->SetPose(tc1w);
    frm->SetPose(tc2w);
    for (size_t idx = 0; idx < common_feat_id.size(); ++idx) {
        if (ptsw[idx][2] > 0) {
            Point::Ptr np(new Point(common_feat_id[idx], ptsw[idx]));
            map_->AddPoint(np);
            last_frm_->EnableObs(common_feat_id[idx]);
            frm->EnableObs(common_feat_id[idx]);
        }
    }
    map_->AddFrame(last_frm_);
    map_->AddFrame(frm);
    last_frm_ = frm;
    return true;
}

} // namespace hityavie