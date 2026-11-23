#ifndef COSIM_COUPLING_H
#define COSIM_COUPLING_H

#include "bitsim/bitsim_types.h"

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace bitsim
{
namespace cosim
{

///
/// \brief Output output of subsystem from feeds input input of subsystem to
///
struct Connection
{
    std::string from;
    uint_t output{0};
    std::string to;
    uint_t input{0};
};

///
/// \brief The subsystems of a co-simulation, their sizes, and which output feeds which input.
///
/// Every input must be fed by exactly one output. The subsystems are numbered in the order they
/// were added; that order is the order of Gauss-Seidel. Every participant of a distributed
/// co-simulation builds the same layout.
///
class CouplingLayout
{
  public:
    ///
    /// \brief Add a subsystem; its index is the number of subsystems before it
    ///
    uint_t add(const std::string &name, uint_t n_inputs, uint_t n_outputs);

    void connect(const std::string &from, uint_t output, const std::string &to, uint_t input);

    ///
    /// \throws std::logic_error if an input is fed by no output, or by more than one
    ///
    void validate() const;

    uint_t size() const noexcept { return static_cast<uint_t>(names_.size()); }

    uint_t index(const std::string &name) const;

    const std::string &name(uint_t i) const { return names_.at(i); }

    uint_t n_inputs(uint_t i) const { return n_inputs_.at(i); }

    uint_t n_outputs(uint_t i) const { return n_outputs_.at(i); }

    const std::vector<Connection> &connections() const noexcept { return connections_; }

    ///
    /// \brief The inputs of every subsystem that the outputs of every subsystem give
    ///
    std::vector<std::vector<real_t>>
    inputs_from(const std::vector<std::vector<real_t>> &outputs) const;

    ///
    /// \brief Every subsystem's inputs, all 0
    ///
    std::vector<std::vector<real_t>> zero_inputs() const;

  private:
    std::vector<std::string> names_;
    std::vector<uint_t> n_inputs_;
    std::vector<uint_t> n_outputs_;
    std::map<std::string, uint_t> index_;
    std::vector<Connection> connections_;
};

///
/// \brief The order in which the subsystems of a macro step take their inputs
///
enum class CouplingScheme
{
    ///
    /// Every subsystem steps at the same time, with the inputs of the start of the step, or
    /// a guess of those of its end. With max_iterations > 1, the step is repeated with the
    /// inputs its outputs gave, until they agree: strong coupling.
    ///
    JACOBI,
    ///
    /// The subsystems step one after the other, in the order of the layout, each with the
    /// outputs at the end of the step of those before it. It cannot step them in parallel.
    ///
    GAUSS_SEIDEL
};

///
/// \brief How a co-simulation couples its subsystems
///
struct CouplingConfig
{
    CouplingScheme scheme{CouplingScheme::JACOBI};
    uint_t max_iterations{1}; ///< JACOBI: the most tries of a macro step; 1 couples loosely
    real_t tolerance{1e-10};  ///< JACOBI: done when no input moves by more than this
    real_t relaxation{1.0};   ///< JACOBI: the next guess is g + relaxation (v - g)
    ///
    /// Guess the inputs at the end of a step by a straight line through those at its start and
    /// at the start of the step before, not by holding them
    ///
    bool extrapolate{false};
};

///
/// \brief The inputs of every subsystem at the present time, and at the time before
///
struct CouplingState
{
    std::vector<std::vector<real_t>> inputs;
    std::vector<std::vector<real_t>> previous;
    bool has_previous{false};
};

///
/// \brief What one macro step did
///
struct MacroStepResult
{
    uint_t iterations{0}; ///< the tries of the step
    real_t residual{0.0}; ///< the largest change of an input in the last try
    bool converged{true}; ///< the residual is within the tolerance; always true for one try
};

///
/// \brief The outputs of every subsystem at the end of try iteration of a macro step, the
/// subsystems having stepped with their inputs moving from begin to end
///
typedef std::function<std::vector<std::vector<real_t>>(
    uint_t iteration, const std::vector<std::vector<real_t>> &begin,
    const std::vector<std::vector<real_t>> &end)>
    try_step_type;

///
/// \brief One macro step of the Jacobi scheme, with as many tries as config allows.
///
/// try_step steps every subsystem, restoring its saved state first if iteration > 0, and
/// returns the outputs of all of them. The next state of the inputs is what the outputs of the
/// last try give.
///
MacroStepResult jacobi_step(const CouplingLayout &layout, const CouplingConfig &config,
                            CouplingState &state, const try_step_type &try_step);

///
/// \brief The inputs at the start: outputs_of(inputs) gives the outputs of every subsystem
/// for the inputs, and resolve() feeds them back as many times as there are subsystems, which
/// is enough unless the direct feedthroughs form a loop.
///
std::vector<std::vector<real_t>> resolve_inputs(
    const CouplingLayout &layout,
    const std::function<std::vector<std::vector<real_t>>(const std::vector<std::vector<real_t>> &)>
        &outputs_of);

} // namespace cosim
} // namespace bitsim

#endif // COSIM_COUPLING_H
