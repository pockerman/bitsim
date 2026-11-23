#ifndef CO_SIMULATION_H
#define CO_SIMULATION_H

#include "bitsim/bitsim_types.h"
#include "bitsim/cosim/cosim_subsystem.h"
#include "bitsim/cosim/coupling.h"
#include "bitsim/utils/concurrency/thread_pool.h"

#include <boost/noncopyable.hpp>

#include <string>
#include <vector>

namespace bitsim
{
namespace cosim
{

///
/// \brief A co-simulation in one process: the master algorithm that steps coupled subsystems,
/// each with its own solver, and passes the outputs of each to the inputs of the others.
///
/// The subsystems only meet every macro step H, so between two meetings each sees the others'
/// outputs as a guess: held from the start of the step, or extrapolated. That is loose
/// coupling, the Jacobi scheme with one try, and its error grows with H; with a stiff coupling,
/// a stiff spring between two bodies for one, it can make the simulation unstable. Strong
/// coupling repeats every macro step, with the inputs that the outputs of the last try gave,
/// until they agree within a tolerance; it costs more tries per step, and allows much longer
/// steps. Gauss-Seidel lets each subsystem use the new outputs of those before it.
///
/// In the Jacobi scheme the subsystems of a try step at the same time, on the threads of a pool
/// if one is set, and the result does not depend on the threads. CoSimParticipant runs the same
/// scheme with every subsystem in a process or on a machine of its own, and gives the same
/// result to the last bit.
///
class CoSimulation : private boost::noncopyable
{
  public:
    explicit CoSimulation(const CouplingConfig &config = CouplingConfig()) : config_(config) {}

    ///
    /// \brief Add a subsystem; it must outlive this
    ///
    void add(const std::string &name, CoSimSubsystem &subsystem);

    void connect(const std::string &from, uint_t output, const std::string &to, uint_t input)
    {
        layout_.connect(from, output, to, input);
    }

    ///
    /// \brief Check the layout and find the inputs at time t0
    ///
    void initialise(real_t t0 = 0.0);

    ///
    /// \brief Advance every subsystem by H
    ///
    MacroStepResult step(real_t H);

    ///
    /// \brief Step the subsystems of a try on the threads of pool; nullptr steps them one
    /// after the other on this thread
    ///
    void set_pool(utils::concurrency::ThreadPool *pool) noexcept { pool_ = pool; }

    real_t time() const noexcept { return t_; }

    const CouplingLayout &layout() const noexcept { return layout_; }

    const CouplingConfig &config() const noexcept { return config_; }

    ///
    /// \brief The inputs of subsystem name at the present time
    ///
    const std::vector<real_t> &inputs(const std::string &name) const;

    ///
    /// \brief The tries of all the macro steps so far
    ///
    uint_t total_iterations() const noexcept { return total_iterations_; }

    ///
    /// \brief The macro steps that ended without converging
    ///
    uint_t n_not_converged() const noexcept { return n_not_converged_; }

  private:
    MacroStepResult gauss_seidel_(real_t H);

    CouplingConfig config_;
    CouplingLayout layout_;
    std::vector<CoSimSubsystem *> subsystems_;
    CouplingState state_;
    utils::concurrency::ThreadPool *pool_{nullptr};
    real_t t_{0.0};
    bool initialised_{false};
    uint_t total_iterations_{0};
    uint_t n_not_converged_{0};
};

} // namespace cosim
} // namespace bitsim

#endif // CO_SIMULATION_H
