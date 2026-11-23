#include "bitsim/cosim/co_simulation.h"
#include "bitsim/utils/concurrency/parallel.h"

#include <stdexcept>

namespace bitsim
{
namespace cosim
{

void CoSimulation::add(const std::string &name, CoSimSubsystem &subsystem)
{
    if (initialised_)
    {
        throw std::logic_error("CoSimulation: add() after initialise()");
    }
    layout_.add(name, subsystem.n_inputs(), subsystem.n_outputs());
    subsystems_.push_back(&subsystem);
}

void CoSimulation::initialise(real_t t0)
{
    layout_.validate();
    if (config_.scheme == CouplingScheme::GAUSS_SEIDEL && config_.max_iterations != 1)
    {
        throw std::invalid_argument("CoSimulation: Gauss-Seidel takes one try per step");
    }
    t_ = t0;
    state_ = CouplingState();
    state_.inputs = resolve_inputs(layout_,
                                   [this](const std::vector<std::vector<real_t>> &u)
                                   {
                                       std::vector<std::vector<real_t>> y(subsystems_.size());
                                       for (std::size_t i = 0; i < y.size(); ++i)
                                       {
                                           y[i] = subsystems_[i]->outputs(u[i]);
                                       }
                                       return y;
                                   });
    initialised_ = true;
}

const std::vector<real_t> &CoSimulation::inputs(const std::string &name) const
{
    return state_.inputs.at(layout_.index(name));
}

MacroStepResult CoSimulation::step(real_t H)
{
    if (!initialised_)
    {
        throw std::logic_error("CoSimulation: call initialise() before step()");
    }
    if (!(H > 0.0))
    {
        throw std::invalid_argument("CoSimulation: the macro step must be positive");
    }
    MacroStepResult result;
    if (config_.scheme == CouplingScheme::GAUSS_SEIDEL)
    {
        result = gauss_seidel_(H);
    }
    else
    {
        const uint_t n = static_cast<uint_t>(subsystems_.size());
        if (config_.max_iterations > 1)
        {
            for (auto *s : subsystems_)
            {
                s->save();
            }
        }
        auto try_step = [&](uint_t it, const std::vector<std::vector<real_t>> &begin,
                            const std::vector<std::vector<real_t>> &end)
        {
            std::vector<std::vector<real_t>> y(n);
            auto one = [&](uint_t i)
            {
                if (it > 0)
                {
                    subsystems_[i]->restore();
                }
                subsystems_[i]->do_step(t_, H, begin[i], end[i]);
                y[i] = subsystems_[i]->outputs(end[i]);
            };
            if (pool_)
            {
                utils::concurrency::parallel_for<uint_t>(0, n, one, *pool_);
            }
            else
            {
                for (uint_t i = 0; i < n; ++i)
                {
                    one(i);
                }
            }
            return y;
        };
        result = jacobi_step(layout_, config_, state_, try_step);
    }
    t_ += H;
    total_iterations_ += result.iterations;
    n_not_converged_ += result.converged ? 0 : 1;
    return result;
}

MacroStepResult CoSimulation::gauss_seidel_(real_t H)
{
    const uint_t n = static_cast<uint_t>(subsystems_.size());
    const auto begin = state_.inputs;
    // the outputs at the end of the step: of the subsystems that stepped, the new ones; of the
    // others, those at the start, held or extrapolated through the inputs they give
    std::vector<std::vector<real_t>> y(n);
    for (uint_t i = 0; i < n; ++i)
    {
        y[i] = subsystems_[i]->outputs(begin[i]);
    }
    auto end = begin;
    for (uint_t i = 0; i < n; ++i)
    {
        // the inputs of i from the outputs known now
        const auto now = layout_.inputs_from(y);
        end[i] = now[i];
        if (config_.extrapolate && state_.has_previous)
        {
            // inputs from subsystems that have not stepped yet are a straight line
            for (const auto &c : layout_.connections())
            {
                if (layout_.index(c.to) == i && layout_.index(c.from) >= i)
                {
                    end[i][c.input] = 2.0 * begin[i][c.input] - state_.previous[i][c.input];
                }
            }
        }
        subsystems_[i]->do_step(t_, H, begin[i], end[i]);
        y[i] = subsystems_[i]->outputs(end[i]);
    }
    state_.previous = state_.inputs;
    state_.has_previous = true;
    state_.inputs = layout_.inputs_from(y);
    MacroStepResult result;
    result.iterations = 1;
    return result;
}

} // namespace cosim
} // namespace bitsim
