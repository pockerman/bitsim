#ifndef SYNCHRONISER_H
#define SYNCHRONISER_H

#include "bitsim/bitsim_types.h"
#include "bitsim/distributed/state_message.h"
#include "bitsim/distributed/transport.h"

#include <boost/noncopyable.hpp>

#include <chrono>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace bitsim
{
namespace distributed
{

///
/// \brief How the participants of a distributed simulation keep time
///
enum class SyncMode
{
    ///
    /// Every participant waits at each step until it has the state of every other at that step.
    /// The simulation is as fast as its slowest participant, and repeatable: every run gives the
    /// same result, however the messages are delayed.
    ///
    LOCKSTEP,

    ///
    /// Every participant keeps pace with the wall clock and uses the newest state it has of each
    /// other, moved forward by the time since it was sent. Nobody waits, so participants that
    /// cannot wait, real robots, can join; the result depends on the delays.
    ///
    WALL_CLOCK
};

///
/// \brief The settings of Synchroniser
///
struct SynchroniserConfig
{
    SyncMode mode{SyncMode::LOCKSTEP};
    real_t sync_period{0.05}; ///< the simulated time between two exchanges [s]

    ///
    /// The other participants. In lockstep every step waits for all of them; in wall-clock mode
    /// they are reported stale until they appear, and participants not named here are accepted
    /// when they appear.
    ///
    std::vector<std::string> peers;

    real_t timeout{10.0};      ///< lockstep: the longest wait for a step [s]
    real_t resend_period{0.1}; ///< lockstep: send the last steps again this often while waiting [s]

    real_t real_time_factor{1.0}; ///< wall-clock: simulated seconds per wall-clock second
    real_t stale_after{0.5};      ///< wall-clock: a state older than this is stale [s]
    bool extrapolate{true};       ///< wall-clock: move the states forward by their age
};

///
/// \brief What a participant knows of another at a step
///
struct PeerState
{
    std::string participant;
    uint_t step{0};
    real_t time{0.0};  ///< the simulated time of the state
    real_t age{0.0};   ///< wall-clock: how long ago it was sent [s]; 0 in lockstep
    bool stale{false}; ///< wall-clock: older than stale_after, or never received
    bool left{false};  ///< the participant has left; bodies is its last state
    std::vector<BodyState> bodies;
    std::vector<real_t> values; ///< the values it sent with the state, as they were
};

///
/// \brief Keeps the participants of a distributed simulation in step.
///
/// Every participant simulates its own robots and calls exchange() every sync_period of
/// simulated time with the state of its bodies. exchange() sends them to the others through the
/// transport and returns what this participant knows of the others, which it then uses to place
/// ghosts of their bodies in its world, to sense them and to avoid them.
///
/// In lockstep (SyncMode::LOCKSTEP) exchange() returns once every peer has sent the same step.
/// While it waits it sends its last two steps again every resend_period, so that a peer that
/// connected late, or lost a message, still gets them; no participant can be more than a step
/// ahead of another, so two are enough. A peer that leave()s is no longer waited for after its
/// last step.
///
/// In wall-clock mode (SyncMode::WALL_CLOCK) exchange() first waits until the wall clock reaches
/// the step, step * sync_period / real_time_factor after the first call, or counts an overrun if
/// it is already late, then sends and returns the newest state of every peer, moved forward
/// with its velocities by its age if extrapolate is set.
///
class Synchroniser : private boost::noncopyable
{
  public:
    Synchroniser(Transport &transport, const std::string &participant,
                 const SynchroniserConfig &config);

    ///
    /// \brief Leaves, if leave() was not called
    ///
    ~Synchroniser();

    ///
    /// \brief Send the state of this participant's bodies at step, and get the others'
    /// \param step the step, counted from 0; every participant exchanges every step
    /// \param bodies the state of this participant's bodies at the step
    /// \param values numbers to send with the bodies, see StateMessage; they are not
    /// extrapolated
    /// \return the peers, sorted by name
    /// \throws std::runtime_error in lockstep if a peer has not sent the step within timeout
    ///
    const std::vector<PeerState> &exchange(uint_t step, const std::vector<BodyState> &bodies,
                                           const std::vector<real_t> &values = {});

    ///
    /// \brief Tell the others that this participant stops after the last step it exchanged
    ///
    void leave();

    real_t time_of(uint_t step) const noexcept { return step * config_.sync_period; }

    const std::string &participant() const noexcept { return participant_; }

    const SynchroniserConfig &config() const noexcept { return config_; }

    uint_t n_sent() const noexcept { return n_sent_; }
    uint_t n_received() const noexcept { return n_received_; }
    uint_t n_resent() const noexcept { return n_resent_; }

    ///
    /// \brief wall-clock: the steps that started more than a sync period late
    ///
    uint_t n_overruns() const noexcept { return n_overruns_; }

    ///
    /// \brief lockstep: the longest exchange() waited [s]; wall-clock: the latest a step started
    ///
    real_t max_wait() const noexcept { return max_wait_; }

  private:
    using steady = std::chrono::steady_clock;

    void send_(const StateMessage &message);
    void handle_(const std::string &bytes);
    void drain_();
    const std::vector<PeerState> &lockstep_(uint_t step);
    const std::vector<PeerState> &wall_clock_(uint_t step);

    Transport &transport_;
    std::string participant_;
    SynchroniserConfig config_;

    std::optional<StateMessage> current_;
    std::optional<StateMessage> previous_;
    bool has_left_{false};

    // lockstep: the states received per peer and step
    std::map<std::string, std::map<uint_t, StateMessage>> pending_;
    // wall-clock: the newest state per peer
    std::map<std::string, StateMessage> latest_;
    // the last step of the peers that left
    std::map<std::string, uint_t> left_at_;

    std::optional<steady::time_point> start_;
    std::vector<PeerState> peers_;

    uint_t n_sent_{0};
    uint_t n_received_{0};
    uint_t n_resent_{0};
    uint_t n_overruns_{0};
    real_t max_wait_{0.0};
};

} // namespace distributed
} // namespace bitsim

#endif // SYNCHRONISER_H
