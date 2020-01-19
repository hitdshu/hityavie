#include "yavie/yavie_estimator.h"
#include "yavie/yavie_frame.h"
#include "yavie/yavie_point.h"
#include "common/geometry_utility.h"

namespace hityavie {

YavieEstimator::YavieEstimator(const YavieParameter &param) {
    cam_.reset(BaseCameraRegisterer::GetInstanceByName(param.cam().type()));
    cam_->Init(param.cam());
    tracker_.reset(BaseTrackerRegisterer::GetInstanceByName(param.tp().type()));
    tracker_->Init(param.tp(), cam_);
    sfm_.reset(new Sfm(cam_, param.sp()));
    rot_calibr_.reset(new RotationCalib());
    gravity_ << 0, 0, param.gravity();
    param_ = param;
    state_ = kYavieInit;
    map_.reset(new YavieMap());
}

void YavieEstimator::ProcessImg(double timestamp, cv::Mat &img) {
    if (kYavieInit == state_) {
        YavieInit(timestamp, img);
        return;
    }
    if (kYavieSfm == state_) {
        YavieSfm(timestamp, img);
        return;
    }
}

void YavieEstimator::ProcessImu(double timestamp, const Eigen::Vector3d &lin_acc, const Eigen::Vector3d &ang_vel) {
    YavieImuData data;
    data.timestamp = timestamp;
    data.lin_acc = lin_acc;
    data.ang_vel = ang_vel;
    timestamp_imu_map_[timestamp] = data;
}

void YavieEstimator::YavieInit(double timestamp, cv::Mat &img) {
    std::vector<Feature> kpts;
    tracker_->Track(img, kpts);
    Frame::Ptr nf(new Frame(kpts));
    sfm_->PushFrame(nf);
    if (kSfmTracking == sfm_->GetState()) {
        state_ = kYavieSfm;
        init_frms_.push_back(FrameWithTime(nf, timestamp));
        last_img_timestamp_ = timestamp;
        last_imu_data_ = GetImuData(timestamp);
    }
}

void YavieEstimator::YavieSfm(double timestamp, cv::Mat &img) {
    std::vector<Feature> kpts;
    tracker_->Track(img, kpts);
    Frame::Ptr nf(new Frame(kpts));
    sfm_->PushFrame(nf);
    init_frms_.push_back(FrameWithTime(nf, timestamp));
    Preintegrator::Ptr pitor(new Preintegrator());
    std::vector<YavieImuData> imu_data = GetImuDataBetweenImages(last_img_timestamp_, timestamp);
    pitor->Init(param_.np(), gravity_, last_imu_data_.lin_acc, last_imu_data_.ang_vel, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    double last_integ_time = last_img_timestamp_;
    for (const auto &id : imu_data) {
        pitor->Integrate(id.timestamp - last_integ_time, id.lin_acc, id.ang_vel);
        last_integ_time = id.timestamp;
    }
    rot_calibr_->Calibrate(sfm_->GetRelativePose().block(0, 0, 3, 3), pitor->GetQ().toRotationMatrix());
    last_img_timestamp_ = timestamp;
    last_imu_data_ = imu_data[imu_data.size() - 1];
    init_ints_.push_back(pitor);
    if (rot_calibr_->IsDone()) {
        state_ = kYavieCalib;
        VisualInertialAlign();
    }
}

void YavieEstimator::VisualInertialAlign() {
    CalcBg();
    LinearAlignment();
}

void YavieEstimator::CalcBg() {
    Eigen::Matrix3d rbc = cam_->GetRic();
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    double en_bef = 0;
    double en_aft = 0;
    for (size_t idx = 0; idx < init_frms_.size() - 1; ++idx) {
        Frame::Ptr f1 = init_frms_[idx].first;
        Frame::Ptr f2 = init_frms_[idx + 1].first;
        Eigen::Matrix3d rc1c2 = f1->GetPose().block(0, 0, 3, 3) * f2->GetPose().inverse().block(0, 0, 3, 3);
        Eigen::Matrix3d rb1b2 = rbc * rc1c2 * rbc.inverse();
        Preintegrator::Ptr preint = init_ints_[idx];
        Eigen::Vector3d tmpb = 2 * (preint->GetQ().inverse() * Eigen::Quaterniond(rb1b2)).vec();
        Eigen::Matrix3d tmpA = preint->GetJacobian().block(6, 12, 3, 3);
        A += tmpA.transpose() * tmpA;
        b += tmpA.transpose() * tmpb;
    }
    Eigen::Vector3d dbg = A.ldlt().solve(b);
    for (auto &pi : init_ints_) {
        Eigen::Vector3d bg = pi->GetBg();
        bg += dbg;
        pi->Reintegrate(Eigen::Vector3d::Zero(), bg);
    }
    state_ = kYavieVialign;
}

void YavieEstimator::LinearAlignment() {
    int num_state = init_frms_.size() * 3 + 3 + 1;
    const double scale_ratio = 50.0;
    const double gravity_ratio = 0.2;
    Eigen::MatrixXd A(num_state, num_state);
    Eigen::VectorXd b(num_state);
    A.setZero();
    b.setZero();
    Eigen::Matrix4d Tbc = cam_->GetTfic();
    Eigen::Matrix4d Tc0w = init_frms_[0].first->GetPose();
    for (size_t idx = 0; idx < init_frms_.size() - 1; ++idx) {
        Frame::Ptr fk = init_frms_[idx].first;
        Frame::Ptr fkp = init_frms_[idx + 1].first;
        Eigen::Matrix4d Tc0ck = Tc0w * fk->GetPose().inverse();
        Eigen::Matrix4d Tc0bk = Tc0ck * Tbc.inverse();
        Eigen::Matrix4d Tckc0 = Tc0ck.inverse();
        Eigen::Matrix4d Tbkc0 = Tc0bk.inverse();
        Eigen::Matrix4d Tc0ckp = Tc0w * fkp->GetPose().inverse();
        Eigen::Matrix4d Tc0bkp = Tc0ckp * Tbc.inverse();
        Eigen::Matrix4d Tckpc0 = Tc0ckp.inverse();
        Eigen::Matrix4d Tbkpc0 = Tc0bkp.inverse();
        Preintegrator::Ptr preint = init_ints_[idx];
        double dt = preint->GetDt();
        Eigen::MatrixXd tmpA(6, 10);
        Eigen::VectorXd tmpb(6);
        tmpA.setZero();
        tmpb.setZero();
        tmpA.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * (-dt);
        tmpA.block(0, 6, 3, 3) = 0.5 * Tbkc0.block(0, 0, 3, 3) * dt * dt / gravity_ratio;
        tmpA.block(0, 9, 3, 1) = Tbkc0.block(0, 0, 3, 3) * (Tc0ckp.block(0, 3, 3, 1) - Tc0ck.block(0, 3, 3, 1)) / scale_ratio;
        tmpA.block(3, 0, 3, 3) = -Eigen::Matrix3d::Identity();
        tmpA.block(3, 3, 3, 3) = (Tbkc0 * Tc0bkp).block(0, 0, 3, 3);
        tmpA.block(3, 6, 3, 3) = Tbkc0.block(0, 0, 3, 3) * dt / gravity_ratio;
        tmpb.block(0, 0, 3, 1) = preint->GetP() - Tbc.block(0, 3, 3, 1) + (Tbkc0 * Tc0bkp).block(0, 0, 3, 3) * Tbc.block(0, 3, 3, 1);
        tmpb.block(3, 0, 3, 1) = preint->GetV();
        Eigen::MatrixXd rA = tmpA.transpose() * tmpA;
        Eigen::VectorXd rb = tmpA.transpose() * tmpb;
        A.block<6, 6>(idx * 3, idx * 3) += rA.topLeftCorner<6, 6>();
        b.segment<6>(idx * 3) += rb.head<6>();
        A.bottomRightCorner<4, 4>() += rA.bottomRightCorner<4, 4>();
        b.tail<4>() += rb.tail<4>();
        A.block<6, 4>(idx * 3, num_state - 4) += rA.topRightCorner<6, 4>();
        A.block<4, 6>(num_state - 4, idx * 3) += rA.bottomLeftCorner<4, 6>();
    }
    Eigen::VectorXd x = A.ldlt().solve(b);
    x.tail<4>().head<3>() /= gravity_ratio;
    x[num_state - 1] /= scale_ratio;
    Eigen::VectorXd rfx = RefineGravity(x);
    rfx = RefineGravity(rfx);
    rfx = RefineGravity(rfx);
    rfx = RefineGravity(rfx);
    VisualInertialInit(rfx);
    state_ = kYavieTracking;
}

Eigen::VectorXd YavieEstimator::RefineGravity(const Eigen::VectorXd &init_x) {
    Eigen::Vector3d g = init_x.tail<4>().head<3>();
    Eigen::Vector3d g0 = g.normalized() * gravity_.norm();
    Eigen::Vector3d xa = g0.normalized();
    Eigen::Vector3d xb;
    Eigen::Vector3d xc;
    Eigen::Vector3d tmp(1, 0, 0);
    if ((xa - tmp).norm() < 1e-5) {
        tmp << 0, 1, 0;
    }
    xb = (tmp - xa * (xa.transpose() * tmp)).normalized();
    xc = xa.cross(xb);
    Eigen::Matrix<double, 3, 2> xbc;
    xbc.block(0, 0, 3, 1) = xb;
    xbc.block(0, 1, 3, 1) = xc;
    int num_state = init_frms_.size() * 3 + 2 + 1;
    const double scale_ratio = 50.0;
    Eigen::MatrixXd A(num_state, num_state);
    Eigen::VectorXd b(num_state);
    A.setZero();
    b.setZero();
    Eigen::Matrix4d Tbc = cam_->GetTfic();
    Eigen::Matrix4d Tc0w = init_frms_[0].first->GetPose();
    for (size_t idx = 0; idx < init_frms_.size() - 1; ++idx) {
        Frame::Ptr fk = init_frms_[idx].first;
        Frame::Ptr fkp = init_frms_[idx + 1].first;
        Eigen::Matrix4d Tc0ck = Tc0w * fk->GetPose().inverse();
        Eigen::Matrix4d Tc0bk = Tc0ck * Tbc.inverse();
        Eigen::Matrix4d Tckc0 = Tc0ck.inverse();
        Eigen::Matrix4d Tbkc0 = Tc0bk.inverse();
        Eigen::Matrix4d Tc0ckp = Tc0w * fkp->GetPose().inverse();
        Eigen::Matrix4d Tc0bkp = Tc0ckp * Tbc.inverse();
        Eigen::Matrix4d Tckpc0 = Tc0ckp.inverse();
        Eigen::Matrix4d Tbkpc0 = Tc0bkp.inverse();
        Preintegrator::Ptr preint = init_ints_[idx];
        double dt = preint->GetDt();
        Eigen::MatrixXd tmpA(6, 9);
        Eigen::VectorXd tmpb(6);
        tmpA.setZero();
        tmpb.setZero();
        tmpA.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * (-dt);
        tmpA.block(0, 6, 3, 2) = 0.5 * Tbkc0.block(0, 0, 3, 3) * dt * dt * xbc;
        tmpA.block(0, 8, 3, 1) = Tbkc0.block(0, 0, 3, 3) * (Tc0ckp.block(0, 3, 3, 1) - Tc0ck.block(0, 3, 3, 1)) / scale_ratio;
        tmpA.block(3, 0, 3, 3) = -Eigen::Matrix3d::Identity();
        tmpA.block(3, 3, 3, 3) = (Tbkc0 * Tc0bkp).block(0, 0, 3, 3);
        tmpA.block(3, 6, 3, 2) = Tbkc0.block(0, 0, 3, 3) * dt * xbc;
        tmpb.block(0, 0, 3, 1) = preint->GetP() - Tbc.block(0, 3, 3, 1) + (Tbkc0 * Tc0bkp).block(0, 0, 3, 3) * Tbc.block(0, 3, 3, 1) - 0.5 * Tbkc0.block(0, 0, 3, 3) * dt * dt * g0;
        tmpb.block(3, 0, 3, 1) = preint->GetV() - Tbkc0.block(0, 0, 3, 3) * dt * g0;
        Eigen::MatrixXd rA = tmpA.transpose() * tmpA;
        Eigen::VectorXd rb = tmpA.transpose() * tmpb;
        A.block<6, 6>(idx * 3, idx * 3) += rA.topLeftCorner<6, 6>();
        b.segment<6>(idx * 3) += rb.head<6>();
        A.bottomRightCorner<3, 3>() += rA.bottomRightCorner<3, 3>();
        b.tail<3>() += rb.tail<3>();
        A.block<6, 3>(idx * 3, num_state - 3) += rA.topRightCorner<6, 3>();
        A.block<3, 6>(num_state - 3, idx * 3) += rA.bottomLeftCorner<3, 6>();
    }
    Eigen::VectorXd x = A.ldlt().solve(b);
    Eigen::Vector2d dg = x.segment<2>(num_state - 3);
    g0 = (g0 + xbc * dg).normalized() * gravity_.norm();
    Eigen::VectorXd nx(num_state + 1);
    nx.block(0, 0, num_state - 3, 1) = x.block(0, 0, num_state - 3, 1);
    nx.segment<3>(num_state - 3) = g0;
    nx[num_state] = x[num_state - 1] / scale_ratio;
    return nx;
}

void YavieEstimator::VisualInertialInit(const Eigen::VectorXd &x) {
    int num_frms = init_frms_.size();
    double scale = x[num_frms * 3 + 3];
    Eigen::Vector3d g = x.block(num_frms * 3, 0, 3, 1);
    Eigen::Vector3d xx;
    Eigen::Vector3d xy;
    Eigen::Vector3d xz = g / g.norm();
    Eigen::Vector3d tmp(1, 0, 0);
    if ((xz - tmp).norm() < 1e-5) {
        tmp << 0, 1, 0;
    }
    xx = (tmp - xz * (xz.transpose() * tmp)).normalized();
    xy = xz.cross(xx);
    Eigen::Matrix3d rc0w;
    rc0w.block(0, 0, 3, 1) = xx;
    rc0w.block(0, 1, 3, 1) = xy;
    rc0w.block(0, 2, 3, 1) = xz;
    Eigen::Matrix3d rb0w = cam_->GetRic() * rc0w;
    Eigen::Matrix4d Tb0w = Eigen::Matrix4d::Identity();
    Tb0w.block(0, 0, 3, 3) = rb0w;
    Eigen::Matrix4d Twb0 = Tb0w.inverse();
    sfm_->GetMap()->Scale(scale);
    Eigen::Matrix4d Tbc = cam_->GetTfic();
    Eigen::Matrix4d Tc0cw = init_frms_[0].first->GetPose();
    for (size_t idx = 0; idx < init_frms_.size(); ++idx) {
        Frame::Ptr frm = init_frms_[idx].first;
        Eigen::Matrix4d Tc0ck = Tc0cw * frm->GetPose().inverse();
        Eigen::Matrix4d Tb0bk = Tbc * Tc0ck * Tbc.inverse();
        Eigen::Matrix4d Twbk = Twb0 * Tb0bk;
        YavieFrame::Ptr nyf;
        if (0 == idx) {
            nyf.reset(new YavieFrame(init_frms_[idx].second, Twbk, frm->GetFeatures(), Preintegrator::Ptr()));
        } else {
            nyf.reset(new YavieFrame(init_frms_[idx].second, Twbk, frm->GetFeatures(), init_ints_[idx - 1]));
        }
        map_->AddFrame(nyf);
    }
    std::vector<Point::Ptr> sfm_pts = sfm_->GetMap()->GetAllPoints();
    std::vector<YavieFrame::Ptr> all_frms = map_->GetAllFrames();
    for (auto &pt : sfm_pts) {
        Eigen::Vector3d pt_cw = pt->GetPosition();
        Eigen::Vector3d pt_w = GeometryUtility::Transform(Twb0 * Tbc * Tc0cw, pt_cw);
        YaviePoint::Ptr np(new YaviePoint(pt->GetId(), pt_w));
        map_->AddPoint(np);
        for (auto &frm : all_frms) {
            if (frm->HasFeature(np->GetId())) {
                Feature ft = frm->GetFeature(np->GetId());
                Eigen::Vector2d proj = cam_->Project(GeometryUtility::Transform((frm->GetPose() * cam_->GetTfic()).inverse(), np->GetPosition()));
                if ((proj - ft.pt_und).norm() < 6) {
                    frm->EnableObs(np->GetId());
                    np->AddObs(frm);
                }
            }
        }
        if (np->GetObsNum() < 2) {
            map_->RmPoint(np->GetId());
        }
    }
}

std::vector<YavieImuData> YavieEstimator::GetImuDataBetweenImages(double timestamp1, double timestamp2) const {
    std::vector<YavieImuData> data;
    auto iter1 = timestamp_imu_map_.upper_bound(timestamp1);
    auto iter2 = timestamp_imu_map_.lower_bound(timestamp2);
    while (iter1 != iter2) {
        data.push_back(iter1->second);
        ++iter1;
    }
    --iter1;
    YavieImuData last = InterpolateImuData(iter1->second, iter2->second, timestamp2);
    data.push_back(last);
    return data;
}

YavieImuData YavieEstimator::InterpolateImuData(const YavieImuData &d1, const YavieImuData &d2, double timestamp) const {
    double lambda = (timestamp - d1.timestamp) / (d2.timestamp - d1.timestamp);
    YavieImuData data;
    data.timestamp = timestamp;
    data.lin_acc = (1 - lambda) * d1.lin_acc + lambda * d2.lin_acc;
    data.ang_vel = (1 - lambda) * d1.ang_vel + lambda * d2.ang_vel;
    return data;
}

YavieImuData YavieEstimator::GetImuData(double timestamp) const {
    auto iter = timestamp_imu_map_.lower_bound(timestamp);
    return iter->second;
}

} // namespace hityavie