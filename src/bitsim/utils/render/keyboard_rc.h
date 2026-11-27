#ifndef BITSIM_UTILS_RENDER_KEYBOARD_RC_H
#define BITSIM_UTILS_RENDER_KEYBOARD_RC_H

/**
 * \file
 * \brief Fly a quadrotor from the keyboard of an Irrlicht window, as with the sticks of a
 * transmitter. Needs bitsim built with Chrono and Irrlicht.
 */

#include "bitsim/bitsim_config.h"

#if defined(BITSIM_IRRLICHT) && defined(BITSIM_CHRONO)

#include "bitsim/bitsim_types.h"
#include "bitsim/control/rc_command.h"

#include <irrlicht.h>

#include <array>

namespace bitsim
{
namespace utils
{
namespace render
{

/**
 * @class KeyboardRC
 * @brief The keyboard as a radio transmitter:
 *
 * | Key | Stick |
 * | --- | ----- |
 * | W, S | throttle up, down: it stays where it is left, as a throttle stick does |
 * | A, D | yaw left, right |
 * | Up, Down | pitch forward, back |
 * | Left, Right | roll left, right |
 * | Space | arm, or disarm |
 * | R | ask the application to start again |
 *
 * The yaw, pitch and roll keys deflect their stick by `deflection` while they are held, and the
 * stick springs back to the centre when they are let go. Add it to the window with
 * chrono::irrlicht::ChVisualSystemIrrlicht::AddUserEventReceiver; it takes these keys, so the
 * camera does not see them.
 */
class KeyboardRC : public irr::IEventReceiver
{
  public:
    ///
    /// @param throttle_rate How fast holding W or S moves the throttle [1/s]
    /// @param deflection How far the yaw, pitch and roll keys move their sticks, in [0, 1]
    ///
    explicit KeyboardRC(real_t throttle_rate = 0.25, real_t deflection = 1.0);

    bool OnEvent(const irr::SEvent &event) override;

    ///
    /// @brief The command for the keys held now, dt [s] after the last call
    ///
    control::RCCommand command(real_t dt);

    ///
    /// @brief True once after R was pressed
    ///
    bool reset_requested();

    ///
    /// @brief Put the throttle down and disarm
    ///
    void reset();

    ///
    /// @brief Hold a key down or let it go, as if it were pressed, for tests and scripts
    ///
    void set_key(irr::EKEY_CODE key, bool down);

  private:
    bool is_down_(irr::EKEY_CODE key) const { return down_[key]; }

    real_t throttle_rate_;
    real_t deflection_;
    real_t throttle_{0.0};
    bool armed_{false};
    bool reset_{false};
    std::array<bool, irr::KEY_KEY_CODES_COUNT> down_{};
};

} // namespace render
} // namespace utils
} // namespace bitsim

#endif
#endif // BITSIM_UTILS_RENDER_KEYBOARD_RC_H
