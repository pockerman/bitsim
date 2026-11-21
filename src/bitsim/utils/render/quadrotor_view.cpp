#include "bitsim/utils/render/quadrotor_view.h"

#if defined(BITSIM_IRRLICHT) && defined(BITSIM_CHRONO)

#include "bitsim/geometry/rotations.h"
#include "bitsim/utils/render/irrlicht_utils.h"

#include <chrono/assets/ChVisualShapeBox.h>
#include <chrono/assets/ChVisualShapeCylinder.h>
#include <chrono/physics/ChBodyEasy.h>

#include <array>
#include <cmath>
#include <stdexcept>

namespace bitsim
{
namespace utils
{
namespace render
{
namespace
{
using chrono::ChVector3d;

// NED, z down, to the window's axes, z up
const chrono::ChMatrix33d NED_TO_Z_UP(ChVector3d(1, 0, 0), ChVector3d(0, -1, 0),
                                      ChVector3d(0, 0, -1));

ChVector3d to_z_up(const RealColVec3d &p) { return {p[0], -p[1], -p[2]}; }
} // namespace

QuadrotorView::QuadrotorView(const std::string &title, const std::vector<Style> &styles,
                             const std::string &record_dir, real_t record_rate)
    : styles_(styles), recorder_(record_dir, record_rate)
{
    if (styles_.empty())
    {
        throw std::invalid_argument("QuadrotorView: a style per drone, at least one");
    }
    system_.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // A floor of 2 m tiles, so that the window shows how fast the drones move
    const int n_tiles = 20;
    const real_t tile = 2.0;
    for (int i = 0; i < n_tiles; ++i)
    {
        for (int j = 0; j < n_tiles; ++j)
        {
            auto t = chrono_types::make_shared<chrono::ChBodyEasyBox>(tile, tile, 0.1, 1000.0, true,
                                                                      false);
            t->SetPos(
                ChVector3d((i - n_tiles / 2 + 0.5) * tile, (j - n_tiles / 2 + 0.5) * tile, -0.05));
            t->SetFixed(true);
            t->GetVisualShape(0)->SetColor((i + j) % 2 == 0 ? chrono::ChColor(0.45f, 0.58f, 0.40f)
                                                            : chrono::ChColor(0.37f, 0.50f, 0.33f));
            system_.Add(t);
        }
    }

    // Posts to see the drones move against, and the start pad
    for (const auto &p :
         std::vector<ChVector3d>{{4, 4, 0}, {4, -4, 0}, {-4, 4, 0}, {-4, -4, 0}, {10, 0, 0}})
    {
        auto post =
            chrono_types::make_shared<chrono::ChBodyEasyBox>(0.2, 0.2, 2.0, 100, true, false);
        post->SetPos(p + ChVector3d(0, 0, 1.0));
        post->SetFixed(true);
        post->GetVisualShape(0)->SetColor(chrono::ChColor(0.85f, 0.55f, 0.15f));
        system_.Add(post);
    }
    auto pad = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.8, 0.8, 0.01, 100, true, false);
    pad->SetPos(ChVector3d(0, 0, 0.005));
    pad->SetFixed(true);
    pad->GetVisualShape(0)->SetColor(chrono::ChColor(0.9f, 0.9f, 0.9f));
    system_.Add(pad);

    // The drones are added before the window is set up: shapes added after it are not shown
    for (const auto &style : styles_)
    {
        drones_.push_back(make_drone_(style.body));
        system_.Add(drones_.back());
        trails_.emplace_back(0.05, 0.0);
    }

    vis_.AttachSystem(&system_);
    vis_.SetWindowSize(1280, 720);
    vis_.SetWindowTitle(title);
    vis_.SetCameraVertical(chrono::CameraVerticalDir::Z);
    vis_.Initialize();
    vis_.AddCamera({-2.5, 0.0, 1.2}, {0.0, 0.0, 0.3});
    // A soft light: the typical lights wash the scene out
    vis_.AddLightDirectional(60.0, 60.0, chrono::ChColor(0.45f, 0.45f, 0.45f),
                             chrono::ChColor(0.05f, 0.05f, 0.05f),
                             chrono::ChColor(0.55f, 0.55f, 0.55f));
}

void QuadrotorView::follow(uint_t drone)
{
    if (drone >= drones_.size())
    {
        throw std::invalid_argument("QuadrotorView::follow: no such drone");
    }
    followed_ = drone;
}

void QuadrotorView::fix_camera(const RealColVec3d &eye_ned, const RealColVec3d &target_ned)
{
    followed_.reset();
    eye_ = to_z_up(eye_ned);
    look_at_ = to_z_up(target_ned);
}

bool QuadrotorView::show(const std::vector<const dynamics::QuadrotorDynamics *> &drones,
                         real_t time, const std::string &status)
{
    if (drones.size() != drones_.size())
    {
        throw std::invalid_argument("QuadrotorView::show: a state per drone of the view");
    }
    if (!vis_.Run())
    {
        return false;
    }

    std::vector<ChVector3d> positions;
    for (uint_t i = 0; i < drones.size(); ++i)
    {
        const auto &d = *drones[i];
        const auto R = geometry::from_yaw_pitch_roll({d.psi(), d.theta(), d.phi()});
        const ChVector3d position = to_z_up(d.get_position());
        drones_[i]->SetPos(position);
        drones_[i]->SetRot(chrono::ChMatrix33d(NED_TO_Z_UP * R * NED_TO_Z_UP));
        trails_[i].add(position);
        positions.push_back(position);
    }

    if (followed_)
    {
        // behind the drone, along its heading
        const auto &d = *drones[*followed_];
        const ChVector3d behind(-std::cos(-d.psi()) * 2.5, -std::sin(-d.psi()) * 2.5, 1.0);
        vis_.UpdateCamera(positions[*followed_] + behind, positions[*followed_]);
    }
    else
    {
        vis_.UpdateCamera(eye_, look_at_);
    }

    vis_.BeginScene();
    vis_.Render();
    for (uint_t i = 0; i < drones.size(); ++i)
    {
        trails_[i].draw(vis_, styles_[i].trail, 1.5);
        irr_draw_line(vis_, positions[i], ChVector3d(positions[i].x(), positions[i].y(), 0),
                      chrono::ChColor(0.2f, 0.2f, 0.2f), 1.0);
    }
    for (const auto &target : targets_)
    {
        const ChVector3d t = to_z_up(target);
        const chrono::ChColor red(0.9f, 0.1f, 0.1f);
        const real_t a = 0.25;
        irr_draw_line(vis_, t - ChVector3d(a, 0, 0), t + ChVector3d(a, 0, 0), red, 2.0);
        irr_draw_line(vis_, t - ChVector3d(0, a, 0), t + ChVector3d(0, a, 0), red, 2.0);
        irr_draw_line(vis_, t - ChVector3d(0, 0, a), t + ChVector3d(0, 0, a), red, 2.0);
        irr_draw_line(vis_, t, ChVector3d(t.x(), t.y(), 0), chrono::ChColor(0.6f, 0.2f, 0.2f), 1.0);
    }
    draw_status_bar(vis_, status);
    vis_.EndScene();
    recorder_.record(vis_, time);
    return true;
}

std::shared_ptr<chrono::ChBody> QuadrotorView::make_drone_(const chrono::ChColor &colour)
{
    auto body = chrono_types::make_shared<chrono::ChBody>();
    body->SetFixed(true);
    const real_t arm = 0.225;
    const auto dark = chrono::ChColor(0.15f, 0.15f, 0.18f);
    auto add = [&](std::shared_ptr<chrono::ChVisualShape> shape, const ChVector3d &at,
                   const chrono::ChColor &c)
    {
        shape->SetColor(c);
        body->AddVisualShape(shape, chrono::ChFrame<>(at));
    };
    add(chrono_types::make_shared<chrono::ChVisualShapeBox>(2 * arm, 0.03, 0.02), {0, 0, 0}, dark);
    add(chrono_types::make_shared<chrono::ChVisualShapeBox>(0.03, 2 * arm, 0.02), {0, 0, 0}, dark);
    add(chrono_types::make_shared<chrono::ChVisualShapeBox>(0.1, 0.1, 0.05), {0, 0, 0}, colour);
    // Rotors 1 front, 2 right, 3 back, 4 left; in the window's axes right is -y
    const std::array<ChVector3d, 4> rotors{ChVector3d(arm, 0, 0.02), ChVector3d(0, -arm, 0.02),
                                           ChVector3d(-arm, 0, 0.02), ChVector3d(0, arm, 0.02)};
    for (std::size_t i = 0; i < rotors.size(); ++i)
    {
        auto disc = chrono_types::make_shared<chrono::ChVisualShapeCylinder>(0.09, 0.005);
        add(disc, rotors[i],
            i == 0 ? chrono::ChColor(0.9f, 0.15f, 0.1f) : chrono::ChColor(0.75f, 0.75f, 0.75f));
    }
    return body;
}

} // namespace render
} // namespace utils
} // namespace bitsim

#endif // BITSIM_IRRLICHT && BITSIM_CHRONO
