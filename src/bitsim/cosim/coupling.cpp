#include "bitsim/cosim/coupling.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bitsim
{
namespace cosim
{

uint_t CouplingLayout::add(const std::string &name, uint_t n_inputs, uint_t n_outputs)
{
    if (index_.count(name))
    {
        throw std::invalid_argument("CouplingLayout: a subsystem " + name + " already exists");
    }
    const uint_t i = size();
    names_.push_back(name);
    n_inputs_.push_back(n_inputs);
    n_outputs_.push_back(n_outputs);
    index_[name] = i;
    return i;
}

uint_t CouplingLayout::index(const std::string &name) const
{
    const auto itr = index_.find(name);
    if (itr == index_.end())
    {
        throw std::invalid_argument("CouplingLayout: no subsystem " + name);
    }
    return itr->second;
}

void CouplingLayout::connect(const std::string &from, uint_t output, const std::string &to,
                             uint_t input)
{
    if (output >= n_outputs(index(from)) || input >= n_inputs(index(to)))
    {
        throw std::invalid_argument("CouplingLayout: " + from + " has no output " +
                                    std::to_string(output) + ", or " + to + " no input " +
                                    std::to_string(input));
    }
    connections_.push_back({from, output, to, input});
}

void CouplingLayout::validate() const
{
    for (uint_t i = 0; i < size(); ++i)
    {
        std::vector<uint_t> fed(n_inputs(i), 0);
        for (const auto &c : connections_)
        {
            if (c.to == names_[i])
            {
                ++fed[c.input];
            }
        }
        for (uint_t k = 0; k < n_inputs(i); ++k)
        {
            if (fed[k] != 1)
            {
                throw std::logic_error("CouplingLayout: input " + std::to_string(k) + " of " +
                                       names_[i] + " is fed by " + std::to_string(fed[k]) +
                                       " outputs, not 1");
            }
        }
    }
}

std::vector<std::vector<real_t>> CouplingLayout::zero_inputs() const
{
    std::vector<std::vector<real_t>> u(size());
    for (uint_t i = 0; i < size(); ++i)
    {
        u[i].assign(n_inputs(i), 0.0);
    }
    return u;
}

std::vector<std::vector<real_t>>
CouplingLayout::inputs_from(const std::vector<std::vector<real_t>> &outputs) const
{
    auto u = zero_inputs();
    for (const auto &c : connections_)
    {
        u[index(c.to)][c.input] = outputs.at(index(c.from)).at(c.output);
    }
    return u;
}

MacroStepResult jacobi_step(const CouplingLayout &layout, const CouplingConfig &config,
                            CouplingState &state, const try_step_type &try_step)
{
    if (config.max_iterations == 0)
    {
        throw std::invalid_argument("jacobi_step: needs at least one try");
    }
    const auto &begin = state.inputs;
    // the first guess of the inputs at the end of the step
    auto guess = begin;
    if (config.extrapolate && state.has_previous)
    {
        for (std::size_t i = 0; i < guess.size(); ++i)
        {
            for (std::size_t k = 0; k < guess[i].size(); ++k)
            {
                guess[i][k] = 2.0 * begin[i][k] - state.previous[i][k];
            }
        }
    }

    MacroStepResult result;
    std::vector<std::vector<real_t>> implied;
    for (uint_t it = 0; it < config.max_iterations; ++it)
    {
        implied = layout.inputs_from(try_step(it, begin, guess));
        result.iterations = it + 1;
        result.residual = 0.0;
        for (std::size_t i = 0; i < guess.size(); ++i)
        {
            for (std::size_t k = 0; k < guess[i].size(); ++k)
            {
                result.residual = std::max(result.residual, std::abs(implied[i][k] - guess[i][k]));
            }
        }
        result.converged = config.max_iterations == 1 || result.residual <= config.tolerance;
        if (result.converged || it + 1 == config.max_iterations)
        {
            break;
        }
        for (std::size_t i = 0; i < guess.size(); ++i)
        {
            for (std::size_t k = 0; k < guess[i].size(); ++k)
            {
                guess[i][k] += config.relaxation * (implied[i][k] - guess[i][k]);
            }
        }
    }
    state.previous = state.inputs;
    state.has_previous = true;
    state.inputs = implied;
    return result;
}

std::vector<std::vector<real_t>> resolve_inputs(
    const CouplingLayout &layout,
    const std::function<std::vector<std::vector<real_t>>(const std::vector<std::vector<real_t>> &)>
        &outputs_of)
{
    auto u = layout.zero_inputs();
    for (uint_t pass = 0; pass < std::max<uint_t>(layout.size(), 1); ++pass)
    {
        u = layout.inputs_from(outputs_of(u));
    }
    return u;
}

} // namespace cosim
} // namespace bitsim
