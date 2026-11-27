#ifndef BITSIM_DISTRIBUTED_CHRONO_GHOSTS_H
#define BITSIM_DISTRIBUTED_CHRONO_GHOSTS_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_CHRONO

#include "bitsim/distributed/synchroniser.h"

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystem.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace bitsim
{
namespace distributed
{

///
/// \brief The state of a Chrono body, to send to the other participants
/// \param size the extent of the body's box along its own axes
///
BodyState chrono_body_state(const std::string &name, const chrono::ChBody &body,
                            const std::array<real_t, 3> &size);

///
/// \brief The bodies of the other participants of a distributed simulation, in this
/// participant's Chrono system.
///
/// A ghost is a fixed box of the body's size, placed where the body is. Fixed, so that Chrono's
/// dynamics do not move it, but it collides: the sensors of this participant's robots see it, and
/// its robots bump into it. update() creates the ghosts of new bodies, moves the others, and
/// removes the ghosts of participants that left. A body without a size is not shown.
///
/// The ghosts do not push back, since this participant does not simulate them: it sees where the
/// others are, not the forces between them. That suits robots that sense and avoid each other;
/// robots that push each other belong in one participant.
///
class CHRONO_Ghosts
{
  public:
    explicit CHRONO_Ghosts(chrono::ChSystem &system) : system_(system) {}

    ~CHRONO_Ghosts();

    ///
    /// \brief Place the ghosts where the peers say their bodies are
    ///
    void update(const std::vector<PeerState> &peers);

    ///
    /// \brief The number of ghosts in the system
    ///
    uint_t size() const noexcept { return static_cast<uint_t>(ghosts_.size()); }

    ///
    /// \brief The ghost of a body of a peer, or nullptr
    ///
    std::shared_ptr<chrono::ChBody> get(const std::string &participant,
                                        const std::string &body) const;

  private:
    chrono::ChSystem &system_;
    std::map<std::string, std::shared_ptr<chrono::ChBodyEasyBox>> ghosts_; // participant/body
};

} // namespace distributed
} // namespace bitsim

#endif
#endif // BITSIM_DISTRIBUTED_CHRONO_GHOSTS_H
