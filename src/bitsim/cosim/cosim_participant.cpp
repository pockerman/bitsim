#include "bitsim/cosim/cosim_participant.h"

#include <stdexcept>
#include <utility>

namespace bitsim
{
namespace cosim
{

CoSimParticipant::CoSimParticipant(CouplingLayout layout, const std::string &name,
                                   CoSimSubsystem &subsystem, distributed::Transport &transport,
                                   const CouplingConfig &coupling,
                                   const CoSimParticipantConfig &config)
    : layout_(std::move(layout)), me_(layout_.index(name)), subsystem_(subsystem),
      coupling_(coupling)
{
    layout_.validate();
    if (coupling_.scheme != CouplingScheme::JACOBI)
    {
        throw std::invalid_argument("CoSimParticipant: only the Jacobi scheme is distributed");
    }
    if (subsystem.n_inputs() != layout_.n_inputs(me_) ||
        subsystem.n_outputs() != layout_.n_outputs(me_))
    {
        throw std::invalid_argument("CoSimParticipant: the subsystem " + name +
                                    " does not have the sizes of the layout");
    }
    distributed::SynchroniserConfig sc;
    sc.mode = distributed::SyncMode::LOCKSTEP;
    sc.timeout = config.timeout;
    sc.resend_period = config.resend_period;
    for (uint_t i = 0; i < layout_.size(); ++i)
    {
        if (i != me_)
        {
            sc.peers.push_back(layout_.name(i));
        }
    }
    sync_ = std::make_unique<distributed::Synchroniser>(transport, name, sc);
}

CoSimParticipant::~CoSimParticipant() = default;

std::vector<std::vector<real_t>> CoSimParticipant::exchange_(const std::vector<real_t> &mine)
{
    const auto &peers = sync_->exchange(round_++, {}, mine);
    std::vector<std::vector<real_t>> all(layout_.size());
    all[me_] = mine;
    for (const auto &p : peers)
    {
        if (p.left || p.stale)
        {
            throw std::runtime_error("CoSimParticipant: " + p.participant + " has left");
        }
        const uint_t i = layout_.index(p.participant);
        if (p.values.size() != layout_.n_outputs(i))
        {
            throw std::runtime_error("CoSimParticipant: " + p.participant +
                                     " sent the wrong number of outputs");
        }
        all[i] = p.values;
    }
    return all;
}

void CoSimParticipant::initialise(real_t t0)
{
    t_ = t0;
    state_ = CouplingState();
    state_.inputs = resolve_inputs(layout_, [this](const std::vector<std::vector<real_t>> &u)
                                   { return exchange_(subsystem_.outputs(u[me_])); });
    initialised_ = true;
}

MacroStepResult CoSimParticipant::step(real_t H)
{
    if (!initialised_)
    {
        throw std::logic_error("CoSimParticipant: call initialise() before step()");
    }
    if (!(H > 0.0))
    {
        throw std::invalid_argument("CoSimParticipant: the macro step must be positive");
    }
    if (coupling_.max_iterations > 1)
    {
        subsystem_.save();
    }
    auto try_step = [&](uint_t it, const std::vector<std::vector<real_t>> &begin,
                        const std::vector<std::vector<real_t>> &end)
    {
        if (it > 0)
        {
            subsystem_.restore();
        }
        subsystem_.do_step(t_, H, begin[me_], end[me_]);
        return exchange_(subsystem_.outputs(end[me_]));
    };
    const auto result = jacobi_step(layout_, coupling_, state_, try_step);
    t_ += H;
    total_iterations_ += result.iterations;
    n_not_converged_ += result.converged ? 0 : 1;
    return result;
}

void CoSimParticipant::leave() { sync_->leave(); }

} // namespace cosim
} // namespace bitsim
