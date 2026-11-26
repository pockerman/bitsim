#ifndef BITSIM_UTILS_RENDER_IRRLICHT_SCENE_H
#define BITSIM_UTILS_RENDER_IRRLICHT_SCENE_H

/**
 * \file irrlicht_scene.h
 * \brief Pieces of an Irrlicht window for a robot simulation: a texture on the floor, a path
 * drawn behind a robot, the view of a camera on the robot in a corner of the window, and a
 * recorder that writes the window to numbered pictures. Needs bitsim built with Chrono and
 * Irrlicht.
 */

#include "bitsim/bitsim_config.h"

#if defined(BITSIM_IRRLICHT) && defined(BITSIM_CHRONO)

#include "bitsim/bitsim_types.h"

#include <chrono/physics/ChBody.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include <string>
#include <vector>

namespace bitsim
{
namespace utils
{
namespace render
{

///
/// @brief Put a picture on the ground: a rectangle of width x height [m] centred on the
/// origin of the world, at height z, with the texture on it. The top left corner of the picture
/// is the corner with the smallest x and the largest y, as for sensors::FloorImage.
///
/// The visual system takes the shapes of the bodies when it is initialised, so call this before
/// ChVisualSystemIrrlicht::Initialize
///
void add_floor_texture(chrono::ChBody &ground, real_t width, real_t height, real_t z,
                       const std::string &texture);

/**
 * @class Trail
 * @brief A path drawn as a line: in the plane, just above the floor, where a robot has been or
 * where it thinks it has been; or in space, where something has flown.
 */
class Trail
{
  public:
    ///
    /// @param min_step Points closer than this to the last one are not added [m]
    /// @param height How high above z = 0 the path is drawn [m]
    ///
    explicit Trail(real_t min_step = 0.02, real_t height = 0.01);

    ///
    /// @brief Add a point, if it is further than min_step from the last one
    ///
    void add(real_t x, real_t y);

    ///
    /// @brief Add a point in space, at its own height, if it is further than min_step from the
    /// last one: the path of something that flies
    ///
    void add(const chrono::ChVector3d &p);

    ///
    /// @brief Draw the path. Call this between Render() and EndScene()
    ///
    void draw(chrono::irrlicht::ChVisualSystemIrrlicht &vis, const chrono::ChColor &colour,
              real_t thickness = 2.0) const;

    std::size_t size() const noexcept { return points_.size(); }
    void clear() { points_.clear(); }

  private:
    real_t min_step_;
    real_t height_;
    std::vector<chrono::ChVector3d> points_;
};

/**
 * @class InsetCamera
 * @brief A camera on a robot whose view is drawn in the top left corner of the window. It only
 * shows the scene; mouse and keyboard still move the main camera.
 */
class InsetCamera
{
  public:
    ///
    /// @param vis The visual system, after Initialize()
    /// @param fov The vertical field of view [rad]
    /// @param width The width of the inset [pixels]
    /// @param height The height of the inset [pixels]
    /// @param label What to write under the inset
    ///
    InsetCamera(chrono::irrlicht::ChVisualSystemIrrlicht &vis, real_t fov, int width, int height,
                const std::string &label = "robot camera");

    ///
    /// @brief Render the scene into the corner, from eye looking at target, both in world
    /// coordinates. Call this between Render() and EndScene()
    ///
    void draw(const chrono::ChVector3d &eye, const chrono::ChVector3d &target);

  private:
    chrono::irrlicht::ChVisualSystemIrrlicht &vis_;
    chrono::irrlicht::RTSCamera *camera_{nullptr};
    irr::scene::ICameraSceneNode *main_camera_{nullptr};
    int width_;
    int height_;
    std::wstring label_;
};

/**
 * @class FrameRecorder
 * @brief Writes the window to dir/frame_00000.png, frame_00001.png, ..., rate times a second of
 * simulation, to be put together into a movie. With an empty dir it writes nothing.
 */
class FrameRecorder
{
  public:
    ///
    /// @param dir Where to write the pictures; it is created if it does not exist
    /// @param rate Pictures per second of simulation
    ///
    FrameRecorder(const std::string &dir, real_t rate = 10.0);

    ///
    /// @brief Write the window if it is time to, at simulation time time [s]. Call this after
    /// EndScene(). Calls may stop for a while, to record only parts of a run
    ///
    void record(chrono::irrlicht::ChVisualSystemIrrlicht &vis, real_t time);

    bool is_recording() const noexcept { return !dir_.empty(); }
    uint_t n_recorded() const noexcept { return n_recorded_; }

  private:
    std::string dir_;
    real_t rate_;
    real_t next_{0.0};
    uint_t n_recorded_{0};
};

///
/// @brief Write text on a light bar along the bottom of the window. Call this between Render()
/// and EndScene()
///
void draw_status_bar(chrono::irrlicht::ChVisualSystemIrrlicht &vis, const std::string &text);

} // namespace render
} // namespace utils
} // namespace bitsim

#endif
#endif // BITSIM_UTILS_RENDER_IRRLICHT_SCENE_H
