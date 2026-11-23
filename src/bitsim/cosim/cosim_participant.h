#ifndef COSIM_PARTICIPANT_H
#define COSIM_PARTICIPANT_H

#include "bitsim/bitsim_types.h"
#include "bitsim/cosim/cosim_subsystem.h"
#include "bitsim/cosim/coupling.h"
#include "bitsim/distributed/synchroniser.h"
#include "bitsim/distributed/transport.h"

#include <boost/noncopyable.hpp>

#include <memory>
#include <string>
#include <vector>

namespace bitsim
{
namespace cosim
{

///
/// \brief The settings of CoSimParticipant
///
struct CoSimParticipantConfig
{
    real_t timeout{10.0};       ///< how long a try waits for the others [s]
    real_t resend_period{0.02}; ///< how often a waiting try sends its outputs again [s]
};

///
/// \brief One subsystem of a co-simulation, in a process or on a machine of its own.
///
/// Every subsystem of the layout is a participant, named after it, with its own
/// CoSimParticipant; they meet through a bitsim::distributed::Transport. Every try of a macro step,
/// a participant steps its subsystem, sends its outputs, and waits for those of the others, with
/// bitsim::distributed::Synchroniser in lockstep. Every participant then has the outputs of every
/// subsystem, and, from the layout that all share, works out the inputs of every subsystem, the
/// residual, and whether to try again, so all agree without another message. The result is that
/// of CoSimulation with the Jacobi scheme in one process, to the last bit.
///
/// Gauss-Seidel would make every participant wait for the ones before it, so it is not offered.
///
class CoSimParticipant : private boost::noncopyable
{
  public:
    ///
    /// \param layout the layout of the whole co-simulation, the same on every participant
    /// \param name the subsystem this participant is
    /// \param subsystem it; it must outlive this
    /// \param transport where the participants meet; it must outlive this
    /// \param coupling how the subsystems couple, the same on every participant; Jacobi only
    /// \param config how long to wait for the others, and how often to send again
    ///
    CoSimParticipant(CouplingLayout layout, const std::string &name, CoSimSubsystem &subsystem,
                     distributed::Transport &transport, const CouplingConfig &coupling = {},
                     const CoSimParticipantConfig &config = {});

    ~CoSimParticipant();

    ///
    /// \brief Find the inputs at time t0, with the others
    ///
    void initialise(real_t t0 = 0.0);

    ///
    /// \brief Advance by H, with the others
    ///
    MacroStepResult step(real_t H);

    ///
    /// \brief Tell the others that this participant stops
    ///
    void leave();

    real_t time() const noexcept { return t_; }

    ///
    /// \brief The inputs of this participant's subsystem at the present time
    ///
    const std::vector<real_t> &inputs() const { return state_.inputs.at(me_); }

    ///
    /// \brief The inputs of subsystem name at the present time: every participant knows them
    ///
    const std::vector<real_t> &inputs(const std::string &name) const
    {
        return state_.inputs.at(layout_.index(name));
    }

    const distributed::Synchroniser &synchroniser() const noexcept { return *sync_; }

    uint_t total_iterations() const noexcept { return total_iterations_; }

    uint_t n_not_converged() const noexcept { return n_not_converged_; }

  private:
    // send my outputs, return everybody's, in the order of the layout
    std::vector<std::vector<real_t>> exchange_(const std::vector<real_t> &mine);

    CouplingLayout layout_;
    uint_t me_;
    CoSimSubsystem &subsystem_;
    CouplingConfig coupling_;
    std::unique_ptr<distributed::Synchroniser> sync_;
    CouplingState state_;
    real_t t_{0.0};
    uint_t round_{0};
    bool initialised_{false};
    uint_t total_iterations_{0};
    uint_t n_not_converged_{0};
};

} // namespace cosim
} // namespace bitsim

#endif // COSIM_PARTICIPANT_H
