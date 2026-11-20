#include "bitsim/utils/render/keyboard_rc.h"

#if defined(BITSIM_IRRLICHT) && defined(BITSIM_CHRONO)

#include <algorithm>

namespace bitsim
{
namespace utils
{
namespace render
{

namespace
{
bool is_flying_key(irr::EKEY_CODE key)
{
    switch (key)
    {
    case irr::KEY_KEY_W:
    case irr::KEY_KEY_S:
    case irr::KEY_KEY_A:
    case irr::KEY_KEY_D:
    case irr::KEY_UP:
    case irr::KEY_DOWN:
    case irr::KEY_LEFT:
    case irr::KEY_RIGHT:
    case irr::KEY_SPACE:
    case irr::KEY_KEY_R:
        return true;
    default:
        return false;
    }
}
} // namespace

KeyboardRC::KeyboardRC(real_t throttle_rate, real_t deflection)
    : throttle_rate_(throttle_rate), deflection_(std::clamp(deflection, 0.0, 1.0))
{
}

void KeyboardRC::set_key(irr::EKEY_CODE key, bool down)
{
    const bool was_down = down_[key];
    down_[key] = down;
    if (down && !was_down)
    {
        if (key == irr::KEY_SPACE)
        {
            armed_ = !armed_;
        }
        else if (key == irr::KEY_KEY_R)
        {
            reset_ = true;
        }
    }
}

bool KeyboardRC::OnEvent(const irr::SEvent &event)
{
    if (event.EventType != irr::EET_KEY_INPUT_EVENT || !is_flying_key(event.KeyInput.Key))
    {
        return false;
    }
    // A held key repeats its pressed event; set_key only acts on the first
    set_key(event.KeyInput.Key, event.KeyInput.PressedDown);
    return true;
}

control::RCCommand KeyboardRC::command(real_t dt)
{
    const real_t up = is_down_(irr::KEY_KEY_W) ? 1.0 : 0.0;
    const real_t down = is_down_(irr::KEY_KEY_S) ? 1.0 : 0.0;
    throttle_ = std::clamp(throttle_ + throttle_rate_ * dt * (up - down), 0.0, 1.0);

    const auto axis = [this](irr::EKEY_CODE plus, irr::EKEY_CODE minus)
    { return deflection_ * ((is_down_(plus) ? 1.0 : 0.0) - (is_down_(minus) ? 1.0 : 0.0)); };

    control::RCCommand command;
    command.throttle = throttle_;
    command.yaw = axis(irr::KEY_KEY_D, irr::KEY_KEY_A);
    command.pitch = axis(irr::KEY_UP, irr::KEY_DOWN);
    command.roll = axis(irr::KEY_RIGHT, irr::KEY_LEFT);
    command.armed = armed_;
    return command;
}

bool KeyboardRC::reset_requested()
{
    const bool requested = reset_;
    reset_ = false;
    return requested;
}

void KeyboardRC::reset()
{
    throttle_ = 0.0;
    armed_ = false;
}

} // namespace render
} // namespace utils
} // namespace bitsim

#endif
