#include "bitrl/utils/render/irrlicht_scene.h"

#if defined(BITRL_IRRLICHT) && defined(BITRL_CHRONO)

#include "bitrl/utils/render/irrlicht_utils.h"

#include <chrono/assets/ChVisualShapeTriangleMesh.h>
#include <chrono/geometry/ChTriangleMeshConnected.h>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace bitrl
{
namespace utils
{
namespace render
{

void add_floor_texture(chrono::ChBody &ground, real_t width, real_t height, real_t z,
                       const std::string &texture)
{
    auto mesh = chrono_types::make_shared<chrono::ChTriangleMeshConnected>();
    const real_t x = 0.5 * width;
    const real_t y = 0.5 * height;
    mesh->GetCoordsVertices() = {{-x, y, z}, {x, y, z}, {x, -y, z}, {-x, -y, z}};
    // Irrlicht's v runs up the picture
    mesh->GetCoordsUV() = {{0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}, {0.0, 0.0}};
    mesh->GetCoordsNormals() = {{0.0, 0.0, 1.0}};
    mesh->GetIndicesVertexes() = {{3, 2, 1}, {3, 1, 0}};
    mesh->GetIndicesUV() = {{3, 2, 1}, {3, 1, 0}};
    mesh->GetIndicesNormals() = {{0, 0, 0}, {0, 0, 0}};

    auto paper = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
    paper->SetMesh(mesh);
    paper->SetTexture(texture);
    ground.AddVisualShape(paper);
}

// ---------------------------------------------------------------------------
// Trail
// ---------------------------------------------------------------------------

Trail::Trail(real_t min_step, real_t height) : min_step_(min_step), height_(height) {}

void Trail::add(real_t x, real_t y)
{
    if (points_.empty() || std::hypot(x - points_.back().x(), y - points_.back().y()) > min_step_)
    {
        points_.emplace_back(x, y, height_);
    }
}

void Trail::add(const chrono::ChVector3d &p)
{
    if (points_.empty() || (p - points_.back()).Length() > min_step_)
    {
        points_.push_back(p);
    }
}

void Trail::draw(chrono::irrlicht::ChVisualSystemIrrlicht &vis, const chrono::ChColor &colour,
                 real_t thickness) const
{
    for (std::size_t i = 1; i < points_.size(); ++i)
    {
        irr_draw_line(vis, points_[i - 1], points_[i], colour, thickness);
    }
}

// ---------------------------------------------------------------------------
// InsetCamera
// ---------------------------------------------------------------------------

InsetCamera::InsetCamera(chrono::irrlicht::ChVisualSystemIrrlicht &vis, real_t fov, int width,
                         int height, const std::string &label)
    : vis_(vis), width_(width), height_(height), label_(label.begin(), label.end())
{
    // A new camera makes itself the active one, so remember the main one
    auto *smgr = vis_.GetSceneManager();
    main_camera_ = vis_.GetActiveCamera();
    camera_ = new chrono::irrlicht::RTSCamera(vis_.GetDevice(), smgr->getRootSceneNode(), smgr, -1);
    camera_->setInputReceiverEnabled(false);
    camera_->setZUp();
    camera_->setFOV(static_cast<irr::f32>(fov));
    camera_->setAspectRatio(static_cast<irr::f32>(width) / height);
    camera_->setNearValue(0.01f);
    camera_->drop();
    smgr->setActiveCamera(main_camera_);
}

void InsetCamera::draw(const chrono::ChVector3d &eye, const chrono::ChVector3d &target)
{
    camera_->setPosition(irr::core::vector3df(eye.x(), eye.y(), eye.z()));
    camera_->setTarget(irr::core::vector3df(target.x(), target.y(), target.z()));

    auto *driver = vis_.GetVideoDriver();
    auto *smgr = vis_.GetSceneManager();
    const irr::core::rect<irr::s32> whole = driver->getViewPort();
    const irr::core::rect<irr::s32> inset(10, 10, 10 + width_, 10 + height_);

    driver->setViewPort(inset);
    driver->clearZBuffer();
    smgr->setActiveCamera(camera_);
    smgr->drawAll();
    smgr->setActiveCamera(main_camera_);
    driver->setViewPort(whole);

    driver->draw2DRectangleOutline(inset, irr::video::SColor(255, 230, 230, 230));
    auto *font = vis_.GetGUIEnvironment()->getSkin()->getFont();
    font->draw(label_.c_str(), irr::core::rect<irr::s32>(14, 14 + height_, 300, 30 + height_),
               irr::video::SColor(255, 255, 255, 255));
}

// ---------------------------------------------------------------------------
// FrameRecorder
// ---------------------------------------------------------------------------

FrameRecorder::FrameRecorder(const std::string &dir, real_t rate) : dir_(dir), rate_(rate)
{
    if (!dir_.empty())
    {
        std::filesystem::create_directories(dir_);
    }
}

void FrameRecorder::record(chrono::irrlicht::ChVisualSystemIrrlicht &vis, real_t time)
{
    if (dir_.empty() || time + 1e-9 < next_)
    {
        return;
    }
    // After a gap, when nothing was recorded for a while, start the schedule again from now
    // rather than catch up with a burst of pictures
    if (time - next_ > 1.0 / rate_)
    {
        next_ = time;
    }
    next_ += 1.0 / rate_;
    std::ostringstream name;
    name << "frame_" << std::setw(5) << std::setfill('0') << n_recorded_++ << ".png";
    vis.WriteImageToFile((std::filesystem::path(dir_) / name.str()).string());
}

// ---------------------------------------------------------------------------
// The status bar
// ---------------------------------------------------------------------------

void draw_status_bar(chrono::irrlicht::ChVisualSystemIrrlicht &vis, const std::string &text)
{
    auto *driver = vis.GetVideoDriver();
    const auto size = driver->getScreenSize();
    const irr::core::rect<irr::s32> bar(0, size.Height - 26, size.Width, size.Height);
    driver->draw2DRectangle(irr::video::SColor(200, 250, 250, 250), bar);
    auto *font = vis.GetGUIEnvironment()->getSkin()->getFont();
    font->draw(std::wstring(text.begin(), text.end()).c_str(),
               irr::core::rect<irr::s32>(10, size.Height - 20, size.Width, size.Height),
               irr::video::SColor(255, 20, 20, 20));
}

} // namespace render
} // namespace utils
} // namespace bitrl

#endif
