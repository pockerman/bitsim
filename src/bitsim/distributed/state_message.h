#ifndef BITSIM_DISTRIBUTED_STATE_MESSAGE_H
#define BITSIM_DISTRIBUTED_STATE_MESSAGE_H

#include "bitsim/bitsim_types.h"

#include <array>
#include <string>
#include <vector>

namespace bitsim
{
namespace distributed
{

///
/// \brief The state of one body of a participant: where it is, how it moves, and how big it is.
///
/// The others use it to draw the body, to sense it, and to avoid it: a participant that receives
/// it from another process can place a ghost of the body in its own world, see
/// CHRONO_Ghosts. The orientation is a unit quaternion {w, x, y, z}; size is the extent of the
/// body's box along its own axes, {0, 0, 0} if unknown.
///
struct BodyState
{
    std::string name;
    std::array<real_t, 3> position{0.0, 0.0, 0.0};
    std::array<real_t, 4> orientation{1.0, 0.0, 0.0, 0.0};
    std::array<real_t, 3> linear_velocity{0.0, 0.0, 0.0};
    std::array<real_t, 3> angular_velocity{0.0, 0.0, 0.0};
    std::array<real_t, 3> size{0.0, 0.0, 0.0};

    ///
    /// \brief A body that moves in the plane: at (x, y), turned yaw about z, moving with (vx, vy)
    /// and turning at wz
    ///
    static BodyState planar(const std::string &name, real_t x, real_t y, real_t yaw,
                            real_t vx = 0.0, real_t vy = 0.0, real_t wz = 0.0,
                            const std::array<real_t, 3> &size = {0.0, 0.0, 0.0});

    ///
    /// \brief The rotation about z of the orientation
    ///
    real_t yaw() const noexcept;

    ///
    /// \brief Where the body would be dt seconds later, moving as it moves now
    ///
    BodyState extrapolated(real_t dt) const;
};

///
/// \brief What a participant of a distributed simulation sends: the state of its bodies at one
/// step, or that it leaves.
///
/// time is the simulated time of the step; stamp is the wall-clock time it was sent, in seconds
/// since the epoch of std::chrono::system_clock, which a participant in wall-clock mode uses to
/// tell how old the state is. Across machines that needs their clocks in step, with NTP for
/// example.
///
/// values carries numbers that are not bodies: the action of an agent whose world every
/// participant simulates, for one, or anything else the participants agree on.
///
struct StateMessage
{
    enum class Kind : uint_t
    {
        STATE = 0, ///< the state of the bodies at step
        LEAVE = 1  ///< the participant stops; its last step was step
    };

    Kind kind{Kind::STATE};
    std::string participant;
    uint_t step{0};
    real_t time{0.0};
    real_t stamp{0.0};
    std::vector<BodyState> bodies;
    std::vector<real_t> values;

    ///
    /// \brief The message in MessagePack, for a transport
    ///
    std::string encode() const;

    ///
    /// \brief Read a message that encode() wrote
    /// \throws std::invalid_argument if bytes is not one
    ///
    static StateMessage decode(const std::string &bytes);
};

///
/// \brief The wall-clock time now, in seconds since the epoch of std::chrono::system_clock
///
real_t wall_time_now();

} // namespace distributed
} // namespace bitsim

#endif // BITSIM_DISTRIBUTED_STATE_MESSAGE_H
