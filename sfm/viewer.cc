#include "sfm/viewer.h"
#include "common/adapter.h"

namespace hityavie {

#define GL_COLOR_PURPLE     (glColor3f(1.0, 0.4, 0.9))
#define GL_COLOR_RED        (glColor3f(1, 0, 0))
#define GL_COLOR_GREEN      (glColor3f(0, 1, 0))
#define GL_COLOR_BLUE       (glColor3f(0, 0, 1))
#define GL_COLOR_WHITE      (glColor3f(1, 1, 1))
#define GL_COLOR_LGREEN     (glColor3f(0.125, 0.698, 0.667))

void Viewer::InitViewer(const Map::Ptr &map) {
    map_ = map;
    vt_ = std::thread(std::bind(&Viewer::Show, this));
}

void Viewer::Show() {
    pangolin::CreateWindowAndBind("Viewer", 1500, 900);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1500, 900, 2000, 2000, 1500, 900, 0.1, 10000),
        pangolin::ModelViewLookAt(20, 20, 100, 0, 0, 0, 0.0, -1.0, 0.0));
    pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1500.0f / 900.0f).SetHandler(new pangolin::Handler3D(s_cam));
    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        DrawCamera();
        DrawPoint();
        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void Viewer::DrawPoint() {
    std::vector<Point::Ptr> all_pts = map_->GetAllPoints();
    glBegin(GL_POINTS);
    GL_COLOR_PURPLE;
    for (size_t idx = 0; idx < all_pts.size(); ++idx) {
        Eigen::Vector3d tp = all_pts[idx]->GetPosition();
        glVertex3f(tp[0], tp[1], tp[2]);
    }
    glEnd();
}

void Viewer::DrawCamera() {
    std::vector<Frame::Ptr> all_frms = map_->GetAllFrames();
    for (const auto &frm : all_frms) {
        pangolin::OpenGlMatrix twc = Adapter::EigenMat2Pangolin(frm->GetPose().inverse());
        glPushMatrix();
        glMultMatrixd(twc.m);
        glLineWidth(2);
        const float &w = 1.2;
        const float h = w * 0.75;
        const float z = w * 0.6;
        GL_COLOR_GREEN;
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
        glVertex3f(w, -h, z);
        glEnd();
        glPopMatrix();
    }
}

} // namespace hityavie