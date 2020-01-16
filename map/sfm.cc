#include "map/sfm.h"
#include "common/adapter.h"
#include "common/geometry_utility.h"

namespace hityavie {

Sfm::Sfm(const Map::Ptr &map, const BaseCamera::Ptr &cam) {
    map_ = map;
    cam_ = cam;
    initer_.reset(new Initializer(cam, map));
    state_ = kSfmEmpty;
}

void Sfm::PushFrame(Frame::Ptr &frm) {
    if (kSfmEmpty == state_) {
        bool flag = initer_->Initialize(frm);
        if (flag) {
            state_ = kSfmInited;
        }
        return;
    } else if (kSfmInited == state_) {
        Frame::Ptr last_frm = map_->GetLastFrame();
        SolvePnp(last_frm, frm);
        if (map_->GetFrameCnt() >= 10) {
            state_ = kSfmDone;
        }
    }
}

void Sfm::SolvePnp(const Frame::Ptr &last_frm, const Frame::Ptr &cur_frm) {
    std::vector<cv::Point2f> pts2d;
    std::vector<cv::Point3f> pts3d;
    std::vector<Feature> feats_lf = last_frm->GetFeatures();
    std::vector<Feature> feats_cf = cur_frm->GetFeatures();
    for (const auto &fl : feats_lf) {
        if (last_frm->IsEffeObs(fl.id)) {
            for (const auto &fc : feats_cf) {
                if (fc.id == fl.id) {
                    pts3d.push_back(Adapter::Vec2Pt(map_->GetPoint(fl.id)->GetPosition()));
                    pts2d.push_back(Adapter::Vec2Pt(fc.pt_und));
                    cur_frm->EnableObs(fc.id);
                    map_->GetPoint(fl.id)->AddObs(cur_frm);
                }
            }
        }
    }
    cv::Mat rv;
    cv::Mat tv;
    cv::solvePnP(pts3d, pts2d, cam_->GetMatK(), cv::Mat(), rv, tv);
    cv::Mat rot;
    cv::Rodrigues(rv, rot);
    Eigen::Matrix3d rcw;
    Eigen::Vector3d tcw;
    rcw << rot.at<float>(0, 0), rot.at<float>(0, 1), rot.at<float>(0, 2), 
        rot.at<float>(1, 0), rot.at<float>(1, 1), rot.at<float>(1, 2), 
        rot.at<float>(2, 0), rot.at<float>(2, 1), rot.at<float>(2, 2);
    tcw << tv.at<float>(0), tv.at<float>(1), tv.at<float>(2);
    Eigen::Matrix4d tfcw;
    tfcw.setIdentity();
    tfcw.block(0, 0, 3, 3) = rcw;
    tfcw.block(0, 3, 3, 1) = tcw;
    cur_frm->SetPose(tfcw);
    map_->AddFrame(cur_frm);
    for (const auto &fl : feats_lf) {
        if (!map_->HasPoint(fl.id)) {
            for (const auto &fc : feats_cf) {
                if (fl.id == fc.id) {
                    Eigen::Vector3d pt = GeometryUtility::Triangulate(last_frm->GetPose(), cam_->Pt2Ray(fl.pt_und), 
                        cur_frm->GetPose(), cam_->Pt2Ray(fc.pt_und));
                    Eigen::Vector3d pt1 = GeometryUtility::Transform(last_frm->GetPose(), pt);
                    Eigen::Vector3d pt2 = GeometryUtility::Transform(cur_frm->GetPose(), pt);
                    if (pt1[2] > 0 && pt2[2] > 0) {
                        Point::Ptr np(new Point(fl.id, pt));
                        map_->AddPoint(np);
                        last_frm->EnableObs(fl.id);
                        cur_frm->EnableObs(fc.id);
                        np->AddObs(last_frm);
                        np->AddObs(cur_frm);
                    }
                }
            }
        }
    }
}

} // namespace hityavie