#include <set>
#include "map/sfm.h"
#include "common/adapter.h"
#include "common/geometry_utility.h"
#include "optim/error_prj.h"
#include "optim/error_dist.h"

namespace hityavie {

Sfm::Sfm(const Map::Ptr &map, const BaseCamera::Ptr &cam, const SfmParam &sp) {
    map_ = map;
    cam_ = cam;
    initer_.reset(new Initializer(cam, map));
    state_ = kSfmEmpty;
    sp_ = sp;
}

void Sfm::PushFrame(Frame::Ptr &frm) {
    if (kSfmEmpty == state_) {
        bool flag = initer_->Initialize(frm);
        if (flag) {
            state_ = kSfmInited;
            std::vector<Frame::Ptr> frms = map_->GetAllFrames();
            twcp_ = frms[1]->GetPose().inverse();
            tcpcc_ = frms[0]->GetPose() * frms[1]->GetPose().inverse();
        }
        return;
    } else if (kSfmInited == state_) {
        frm->SetPose(twcp_.inverse());
        Frame::Ptr last_frm = map_->GetLastFrame();
        SolvePnp(last_frm, frm);
        if (frm->GetEffObsNum() < sp_.min_eff_obs_num() || (frm->GetId() - last_frm->GetId()) >= sp_.max_frm_interval()) {
            AddKeyFrame(last_frm, frm);
        }
        twcp_ = frm->GetPose().inverse();
        tcpcc_ = last_frm->GetPose() * frm->GetPose().inverse();
    }
}

void Sfm::SolvePnp(const Frame::Ptr &last_frm, const Frame::Ptr &cur_frm) {
    std::vector<Eigen::Vector2d> vs2d;
    std::vector<Eigen::Vector3d> vs3d;
    std::vector<int> fids;
    std::vector<Feature> feats_lf = last_frm->GetFeatures();
    std::vector<Feature> feats_cf = cur_frm->GetFeatures();
    for (const auto &fl : feats_lf) {
        if (last_frm->IsEffeObs(fl.id)) {
            for (const auto &fc : feats_cf) {
                if (fc.id == fl.id) {
                    vs2d.push_back(fc.pt_und);
                    vs3d.push_back(map_->GetPoint(fl.id)->GetPosition());
                    fids.push_back(fl.id);
                }
            }
        }
    }
    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(2);
    ceres::LocalParameterization *quat_ptr = new ceres::QuaternionParameterization();
    Eigen::Matrix4d tcw = cur_frm->GetPose();
    Eigen::Vector4d rv = Adapter::Rot2Quat(tcw.block(0, 0, 3, 3));
    Eigen::Vector3d tv = tcw.block(0, 3, 3, 1);
    Eigen::Matrix3d k = Adapter::Cvk2Matk(cam_->GetMatK());
    for (size_t idx = 0; idx < vs2d.size(); ++idx) {
        Eigen::Vector2d &pt_obs = vs2d[idx];
        Eigen::Vector3d &pt_3d = vs3d[idx];
        Eigen::Vector2d sqrt_info(1, 1);
        ceres::CostFunction *cost_func = ErrorPrj::Create(pt_obs, sqrt_info, k);
        problem.AddResidualBlock(cost_func, loss_function, &rv[0], &tv[0], &pt_3d[0]);
        problem.SetParameterization(&rv[0], quat_ptr);
        problem.SetParameterBlockConstant(&pt_3d[0]);
    }
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    tcw.block(0, 0, 3, 3) = Adapter::Quat2Rot(rv);
    tcw.block(0, 3, 3, 1) = tv;
    cur_frm->SetPose(tcw);
    for (size_t idx = 0; idx < fids.size(); ++idx) {
        Eigen::Vector2d &pt_obs = vs2d[idx];
        Eigen::Vector3d &pt_3d = vs3d[idx];
        Eigen::Vector2d pt_prj = cam_->Project(GeometryUtility::Transform(tcw, pt_3d));
        if ((pt_obs - pt_prj).norm() < 10) {
            cur_frm->EnableObs(fids[idx]);
        }
    }
}

void Sfm::AddKeyFrame(const Frame::Ptr &last_frm, const Frame::Ptr &cur_frm) {
    std::vector<Feature> feats_lf = last_frm->GetFeatures();
    std::vector<Feature> feats_cf = cur_frm->GetFeatures();
    for (const auto &fl : feats_lf) {
        if (!map_->HasPoint(fl.id)) {
            for (const auto &fc : feats_cf) {
                if (fl.id == fc.id) {
                    Eigen::Vector3d pt = GeometryUtility::Triangulate(last_frm->GetPose(), cam_->Pt2Ray(fl.pt_und), cur_frm->GetPose(), cam_->Pt2Ray(fc.pt_und));
                    Eigen::Vector3d pt1 = GeometryUtility::Transform(last_frm->GetPose(), pt);
                    Eigen::Vector3d pt2 = GeometryUtility::Transform(cur_frm->GetPose(), pt);
                    if (pt1[2] > 0 && pt2[2] > 0) {
                        Point::Ptr np(new Point(fl.id, pt));
                        map_->AddPoint(np);
                        last_frm->EnableObs(fl.id);
                        cur_frm->EnableObs(fc.id);
                        np->AddObs(last_frm);
                    }
                }
            }
        }
    }
    for (const auto &feat : feats_cf) {
        if (cur_frm->IsEffeObs(feat.id)) {
            map_->GetPoint(feat.id)->AddObs(cur_frm);
        }
    }
    map_->AddFrame(cur_frm);
    if (map_->GetFrameCnt() < 7) {
        GlobalOptimize();
    } else {
        LocalOptimize();
    }
}

void Sfm::LocalOptimize() {
    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(2);
    ceres::LocalParameterization *quat_ptr = new ceres::QuaternionParameterization();
    Eigen::Matrix3d k = Adapter::Cvk2Matk(cam_->GetMatK());
    std::vector<Frame::Ptr> all_frms = map_->GetAllFrames();
    std::set<Frame::Ptr> inner_frms(all_frms.end() - std::min((int)all_frms.size(), (int)sp_.local_win_size()), all_frms.end());
    std::set<Point::Ptr> inner_pts;
    std::set<Frame::Ptr> outer_frms;
    for (auto iter = inner_frms.begin(); iter != inner_frms.end(); ++iter) {
        std::vector<Feature> feats = (*iter)->GetFeatures();
        for (const auto &feat: feats) {
            if (!(*iter)->IsEffeObs(feat.id)) {
                continue;
            }
            inner_pts.insert(map_->GetPoint(feat.id));
        }
    } 
    for (auto iter = inner_pts.begin(); iter != inner_pts.end(); ++iter) {
        std::vector<Frame::Ptr> frms = (*iter)->GetAllObs();
        for (const auto &frm : frms) {
            if (!inner_frms.count(frm)) {
                outer_frms.insert(frm);
            }
        }
    }
    std::map<int, Eigen::Vector4d> id_rv_inner;
    std::map<int, Eigen::Vector3d> id_tv_inner;
    std::map<int, Eigen::Vector4d> id_rv_outer;
    std::map<int, Eigen::Vector3d> id_tv_outer;
    std::map<int, Eigen::Vector3d> id_pt;
    for (const auto &frm : inner_frms) {
        Eigen::Matrix4d tcw = frm->GetPose();
        Eigen::Vector4d rv = Adapter::Rot2Quat(tcw.block(0, 0, 3, 3));
        Eigen::Vector3d tv = tcw.block(0, 3, 3, 1);
        id_rv_inner[frm->GetId()] = rv;
        id_tv_inner[frm->GetId()] = tv;
    }
    for (const auto &frm : outer_frms) {
        Eigen::Matrix4d tcw = frm->GetPose();
        Eigen::Vector4d rv = Adapter::Rot2Quat(tcw.block(0, 0, 3, 3));
        Eigen::Vector3d tv = tcw.block(0, 3, 3, 1);
        id_rv_outer[frm->GetId()] = rv;
        id_tv_outer[frm->GetId()] = tv;
    }
    for (const auto &pt : inner_pts) {
        id_pt[pt->GetId()] = pt->GetPosition();
    }
    for (const auto &frm : inner_frms) {
        std::vector<Feature> feats = frm->GetFeatures();
        Eigen::Vector4d &rv = id_rv_inner[frm->GetId()];
        Eigen::Vector3d &tv = id_tv_inner[frm->GetId()];
        for (const auto &feat: feats) {
            if (!frm->IsEffeObs(feat.id)) {
                continue;
            }
            Eigen::Vector2d pt_obs = feat.pt_und;
            Eigen::Vector3d &pt_3d = id_pt[feat.id];
            Eigen::Vector2d sqrt_info(1, 1);
            ceres::CostFunction *cost_func = ErrorPrj::Create(pt_obs, sqrt_info, k);
            problem.AddResidualBlock(cost_func, loss_function, &rv[0], &tv[0], &pt_3d[0]);
            problem.SetParameterization(&rv[0], quat_ptr);
        }
    }
    for (const auto &frm : outer_frms) {
        std::vector<Feature> feats = frm->GetFeatures();
        Eigen::Vector4d &rv = id_rv_outer[frm->GetId()];
        Eigen::Vector3d &tv = id_tv_outer[frm->GetId()];
        for (const auto &feat: feats) {
            if (!frm->IsEffeObs(feat.id)) {
                continue;
            }
            if (inner_pts.find(map_->GetPoint(feat.id)) == inner_pts.end()) {
                continue;
            }
            Eigen::Vector2d pt_obs = feat.pt_und;
            Eigen::Vector3d &pt_3d = id_pt[feat.id];
            Eigen::Vector2d sqrt_info(1, 1);
            ceres::CostFunction *cost_func = ErrorPrj::Create(pt_obs, sqrt_info, k);
            problem.AddResidualBlock(cost_func, loss_function, &rv[0], &tv[0], &pt_3d[0]);
            problem.SetParameterization(&rv[0], quat_ptr);
        }
        problem.SetParameterBlockConstant(&rv[0]);
        problem.SetParameterBlockConstant(&tv[0]);
    }
    int ff_id = all_frms[0]->GetId();
    if (id_tv_inner.count(ff_id)) {
        Eigen::Vector4d &rv = id_rv_inner[ff_id];
        Eigen::Vector3d &tv = id_tv_inner[ff_id];
        problem.SetParameterBlockConstant(&rv[0]);
        problem.SetParameterBlockConstant(&tv[0]);
    }
    int sf_id = all_frms[1]->GetId();
    if (id_tv_inner.count(sf_id)) {
        Eigen::Vector3d &tv = id_tv_inner[sf_id];
        ceres::CostFunction *cost_func = ErrorDist::Create(1.0, 1e4);
        problem.AddResidualBlock(cost_func, nullptr, &tv[0]);
    }
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    for (const auto &frm : inner_frms) {
        Eigen::Matrix4d tcw = frm->GetPose();
        Eigen::Vector4d &rv = id_rv_inner[frm->GetId()];
        Eigen::Vector3d &tv = id_tv_inner[frm->GetId()];
        tcw.block(0, 0, 3, 3) = Adapter::Quat2Rot(rv);
        tcw.block(0, 3, 3, 1) = tv;
        frm->SetPose(tcw);
    }
    for (const auto &pt : inner_pts) {
        pt->SetPosition(id_pt[pt->GetId()]);
    }
}

void Sfm::GlobalOptimize() {
    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(2);
    ceres::LocalParameterization *quat_ptr = new ceres::QuaternionParameterization();
    Eigen::Matrix3d k = Adapter::Cvk2Matk(cam_->GetMatK());
    std::vector<Frame::Ptr> all_frms = map_->GetAllFrames();
    std::vector<Point::Ptr> all_pts = map_->GetAllPoints();
    std::map<int, Eigen::Vector4d> id_rv;
    std::map<int, Eigen::Vector3d> id_tv;
    std::map<int, Eigen::Vector3d> id_pt;
    for (const auto &frm : all_frms) {
        Eigen::Matrix4d tcw = frm->GetPose();
        Eigen::Vector4d rv = Adapter::Rot2Quat(tcw.block(0, 0, 3, 3));
        Eigen::Vector3d tv = tcw.block(0, 3, 3, 1);
        id_rv[frm->GetId()] = rv;
        id_tv[frm->GetId()] = tv;
    }
    for (const auto &pt : all_pts) {
        id_pt[pt->GetId()] = pt->GetPosition();
    }
    for (const auto &frm : all_frms) {
        std::vector<Feature> feats = frm->GetFeatures();
        Eigen::Vector4d &rv = id_rv[frm->GetId()];
        Eigen::Vector3d &tv = id_tv[frm->GetId()];
        for (const auto &feat: feats) {
            if (!frm->IsEffeObs(feat.id)) {
                continue;
            }
            Eigen::Vector2d pt_obs = feat.pt_und;
            Eigen::Vector3d &pt_3d = id_pt[feat.id];
            Eigen::Vector2d sqrt_info(1, 1);
            ceres::CostFunction *cost_func = ErrorPrj::Create(pt_obs, sqrt_info, k);
            problem.AddResidualBlock(cost_func, loss_function, &rv[0], &tv[0], &pt_3d[0]);
            problem.SetParameterization(&rv[0], quat_ptr);
        }
    }
    Eigen::Vector4d &rv_ff = id_rv[all_frms[0]->GetId()];
    Eigen::Vector3d &tv_ff = id_tv[all_frms[0]->GetId()];
    problem.SetParameterBlockConstant(&rv_ff[0]);
    problem.SetParameterBlockConstant(&tv_ff[0]);
    Eigen::Vector3d &tv_sf = id_tv[all_frms[1]->GetId()];
    ceres::CostFunction *cost_func = ErrorDist::Create(1.0, 1e4);
    problem.AddResidualBlock(cost_func, nullptr, &tv_sf[0]);
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    for (const auto &frm : all_frms) {
        Eigen::Matrix4d tcw = frm->GetPose();
        Eigen::Vector4d &rv = id_rv[frm->GetId()];
        Eigen::Vector3d &tv = id_tv[frm->GetId()];
        tcw.block(0, 0, 3, 3) = Adapter::Quat2Rot(rv);
        tcw.block(0, 3, 3, 1) = tv;
        frm->SetPose(tcw);
    }
    for (const auto &pt : all_pts) {
        pt->SetPosition(id_pt[pt->GetId()]);
    }
}

} // namespace hityavie