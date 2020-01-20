#include "yavie/viewer.h"
#include "common/adapter.h"

namespace hityavie {

#define GL_COLOR_PURPLE     (glColor3f(1.0, 0.4, 0.9))
#define GL_COLOR_RED        (glColor3f(1, 0, 0))
#define GL_COLOR_GREEN      (glColor3f(0, 1, 0))
#define GL_COLOR_BLUE       (glColor3f(0, 0, 1))
#define GL_COLOR_WHITE      (glColor3f(1, 1, 1))
#define GL_COLOR_LGREEN     (glColor3f(0.125, 0.698, 0.667))

void Viewer::InitViewer(const YavieMap::Ptr &map, const BaseCamera::Ptr cam) {
    map_ = map;
    cam_ = cam;
    vt_ = std::thread(std::bind(&Viewer::Show, this));
    is_running_.store(true);
}

void Viewer::Close() {
    is_running_.store(false);
    vt_.join();
}

void Viewer::Show() {
    pangolin::CreateWindowAndBind("Viewer", 900, 600);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(900, 600, 400, 400, 450, 300, 0.1, 300),
        pangolin::ModelViewLookAt(-2, -2, 6, 0, 0, 0, 0.0, 1.0, 1.0));
    pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -900.0f/600.0f).SetHandler(new pangolin::Handler3D(s_cam));
    pangolin::OpenGlMatrix twc;
    twc.SetIdentity();
    while (is_running_.load() && pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        DrawCamera();
        DrawPoint();
        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void Viewer::DrawPoint() {
    if (map_) {
        // YavieFrame::Ptr cf = map_->GetCurFrame();
        // if (cf) {
        //     glPointSize(2);
        //     glBegin(GL_POINTS);
        //     GL_COLOR_GREEN;
        //     std::vector<Feature> fts = cf->GetFeatures();
        //     for (auto &ft : fts) {
        //         if (!map_->HasPoint(ft.id)) {
        //             continue;
        //         }
        //         YaviePoint::Ptr pt = map_->GetPoint(ft.id);
        //         if (pt) {
        //             Eigen::Vector3d tp = pt->GetPosition();
        //             glVertex3f(tp[0], tp[1], tp[2]);
        //         }
        //     }
        //     glEnd();
        // }
        std::vector<YaviePoint::Ptr> all_pts = map_->GetAllPoints();
        glPointSize(1);
        glBegin(GL_POINTS);
        GL_COLOR_PURPLE;
        for (size_t idx = 0; idx < all_pts.size(); ++idx) {
            if (all_pts[idx]->GetObsNum() > 4) {
                Eigen::Vector3d tp = all_pts[idx]->GetPosition();
                glVertex3f(tp[0], tp[1], tp[2]);
            }
        }
        glEnd();
    }
}

void Viewer::DrawCamera() {
    if (map_) {
        std::vector<YavieFrame::Ptr> all_frms = map_->GetAllFrames();
        YavieFrame::Ptr cf = map_->GetCurFrame();
        if (cf) {
            all_frms.push_back(cf);
        }
        for (const auto &frm : all_frms) {
            pangolin::OpenGlMatrix twc = Adapter::EigenMat2Pangolin(frm->GetPose() * cam_->GetTfic());
            glPushMatrix();
            glMultMatrixd(twc.m);
            glLineWidth(2);
            float w = 0.05;
            float h = w * 0.75;
            float z = w * 0.6;
            GL_COLOR_BLUE;
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);
            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);
            glVertex3f(-w, -h, z);
            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();
            glPopMatrix();
        }
        glLineWidth(1);
        glBegin(GL_LINES);
        GL_COLOR_BLUE;
        for (size_t idx = 0; idx < all_frms.size(); ++idx) {
            YavieFrame::Ptr frm = all_frms[idx];
            Eigen::Matrix4d twc = frm->GetPose() * cam_->GetTfic();
            Eigen::Vector3d position = twc.block(0, 3, 3, 1);
            YavieFrame::Ptr next_frm = all_frms[std::min(idx + 1, all_frms.size() - 1)];
            Eigen::Matrix4d next_twc = next_frm->GetPose() * cam_->GetTfic();
            Eigen::Vector3d next_position = next_twc.block(0, 3, 3, 1);
            glVertex3f(position[0], position[1], position[2]);
            glVertex3f(next_position[0], next_position[1], next_position[2]);
        }
        glEnd();
    }
}

} // namespace hityavie