#include "bitsim/cosim/cosim_subsystem.h"

#include <stdexcept>
#include <utility>

namespace bitsim
{
namespace cosim
{

OdeSubsystem::OdeSubsystem(std::vector<real_t> x0, uint_t n_inputs, uint_t n_outputs, rhs_type f,
                           output_type h, uint_t n_substeps)
    : x_(std::move(x0)), saved_(x_), n_inputs_(n_inputs), n_outputs_(n_outputs), f_(std::move(f)),
      h_(std::move(h)), n_substeps_(n_substeps)
{
    if (!f_ || !h_ || n_substeps_ == 0)
    {
        throw std::invalid_argument("OdeSubsystem: needs f, h and at least one substep");
    }
}

std::vector<real_t> OdeSubsystem::outputs(const std::vector<real_t> &inputs) const
{
    auto y = h_(x_, inputs);
    if (y.size() != n_outputs_)
    {
        throw std::runtime_error("OdeSubsystem: h gave the wrong number of outputs");
    }
    return y;
}

void OdeSubsystem::do_step(real_t t, real_t H, const std::vector<real_t> &u_begin,
                           const std::vector<real_t> &u_end)
{
    if (u_begin.size() != n_inputs_ || u_end.size() != n_inputs_)
    {
        throw std::invalid_argument("OdeSubsystem: wrong number of inputs");
    }
    const real_t h = H / static_cast<real_t>(n_substeps_);
    const std::size_t n = x_.size();
    std::vector<real_t> u(n_inputs_), tmp(n);

    // the inputs at time s of the macro step
    auto inputs_at = [&](real_t s)
    {
        const real_t a = H > 0.0 ? (s - t) / H : 0.0;
        for (uint_t i = 0; i < n_inputs_; ++i)
        {
            u[i] = u_begin[i] + a * (u_end[i] - u_begin[i]);
        }
        return u;
    };
    auto add = [&](const std::vector<real_t> &k, real_t c)
    {
        for (std::size_t j = 0; j < n; ++j)
        {
            tmp[j] = x_[j] + c * k[j];
        }
        return tmp;
    };

    for (uint_t s = 0; s < n_substeps_; ++s)
    {
        const real_t ts = t + s * h;
        const auto k1 = f_(ts, x_, inputs_at(ts));
        const auto k2 = f_(ts + 0.5 * h, add(k1, 0.5 * h), inputs_at(ts + 0.5 * h));
        const auto k3 = f_(ts + 0.5 * h, add(k2, 0.5 * h), inputs_at(ts + 0.5 * h));
        const auto k4 = f_(ts + h, add(k3, h), inputs_at(ts + h));
        for (std::size_t j = 0; j < n; ++j)
        {
            x_[j] += h / 6.0 * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]);
        }
    }
}

} // namespace cosim
} // namespace bitsim
