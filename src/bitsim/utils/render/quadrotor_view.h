#ifndef BITSIM_UTILS_RENDER_QUADROTOR_VIEW_H
#define BITSIM_UTILS_RENDER_QUADROTOR_VIEW_H

/**
 * \file quadrotor_view.h
 * \brief An Irrlicht window that shows quadrotors flying over a tiled floor. Needs bitsim built
 * with Chrono and Irrlicht.
 */

#include "bitsim/bitsim_config.h"

#if defined(BITSIM_IRRLICHT) && defined(BITSIM_CHRONO)

#include "bitsim/bitsim_types.h"
#include "bitsim/dynamics/quadrotor_dynamics.h"
#include "bitsim/utils/render/irrlicht_scene.h"

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace bitsim
{
namespace utils
{
namespace render
{

///
/// @brief How a drone of a QuadrotorView looks: the colour of its body and of its trail
///
struct QuadrotorStyle
{
    chrono::ChColor body{0.25f, 0.45f, 0.8f};
    chrono::ChColor trail{1.0f, 0.9f, 0.2f};
};

/**
 * @class QuadrotorView
 * @brief A window with quadrotors over a floor of 2 m tiles, with posts to see them move against
 * and a start pad. Each drone is two arms in a cross, a body and four rotors, the front one red,
 * and leaves a trail. The positions and the angles come from
 * dynamics::QuadrotorDynamics, whose world is NED; the window's is z up.
 *
 * The camera follows one of the drones from behind, along its heading, or looks from a fixed
 * place. Targets, drawn as crosses with a line down to the floor, show where a controller is
 * taking the drones. With a record directory, a FrameRecorder writes the window there.
 */
class QuadrotorView
{
  public:
    using Style = QuadrotorStyle;

    ///
    /// @param title The title of the window
    /// @param styles One per drone
    /// @param record_dir Where to write the window, or empty for nowhere
    /// @param record_rate Pictures per second of simulation
    ///
    QuadrotorView(const std::string &title,
                  const std::vector<QuadrotorStyle> &styles = {QuadrotorStyle()},
                  const std::string &record_dir = "", real_t record_rate = 10.0);

    ///
    /// @brief Follow a drone from behind; the default, with the first drone
    ///
    void follow(uint_t drone);

    ///
    /// @brief Look from eye at target, both in NED [m], instead of following a drone
    ///
    void fix_camera(const RealColVec3d &eye_ned, const RealColVec3d &target_ned);

    ///
    /// @brief Show targets at the given NED positions [m]; none by default
    ///
    void set_targets(const std::vector<RealColVec3d> &targets_ned) { targets_ = targets_ned; }

    ///
    /// @brief Draw the drones, one per style, at simulation time [s], with a status line
    /// @return False once the window has been closed
    ///
    bool show(const std::vector<const dynamics::QuadrotorDynamics *> &drones, real_t time,
              const std::string &status);

    chrono::irrlicht::ChVisualSystemIrrlicht &vis() { return vis_; }
    const FrameRecorder &recorder() const noexcept { return recorder_; }

  private:
    std::shared_ptr<chrono::ChBody> make_drone_(const chrono::ChColor &colour);

    chrono::ChSystemNSC system_;
    std::vector<Style> styles_;
    std::vector<std::shared_ptr<chrono::ChBody>> drones_;
    std::vector<Trail> trails_;
    chrono::irrlicht::ChVisualSystemIrrlicht vis_;
    FrameRecorder recorder_;
    std::optional<uint_t> followed_{0};
    chrono::ChVector3d eye_;
    chrono::ChVector3d look_at_;
    std::vector<RealColVec3d> targets_;
};

} // namespace render
} // namespace utils
} // namespace bitsim

#endif // BITSIM_IRRLICHT && BITSIM_CHRONO

#endif // BITSIM_UTILS_RENDER_QUADROTOR_VIEW_H
