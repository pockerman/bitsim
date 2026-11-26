#ifndef BITSIM_COSIM_COSIM_SUBSYSTEM_H
#define BITSIM_COSIM_COSIM_SUBSYSTEM_H

#include "bitsim/bitsim_types.h"

#include <functional>
#include <vector>

namespace bitsim
{
namespace cosim
{

///
/// \brief A part of a coupled system that a co-simulation steps on its own: a robot, a trailer,
/// a motor, each with its own solver, coupled to the others only through its inputs and outputs.
///
/// The interface follows that of a co-simulation unit of the Functional Mock-up Interface: the
/// master gives the inputs, asks the subsystem to advance by a macro step, and reads the outputs.
/// Over a step from t to t + H the inputs move linearly from u_begin to u_end, so a subsystem can
/// interpolate them within its own smaller steps; a master that does not know u_end yet passes
/// a guess.
///
/// To couple tightly, the master repeats a macro step until the inputs and the outputs agree.
/// It calls save() before the first try and restore() before every other, so the subsystem must
/// be able to go back to the state it saved.
///
class CoSimSubsystem
{
  public:
    virtual ~CoSimSubsystem() = default;

    virtual uint_t n_inputs() const = 0;

    virtual uint_t n_outputs() const = 0;

    ///
    /// \brief The outputs at the present time, with the inputs at the present time.
    ///
    /// Outputs that depend on the inputs directly, a force computed from a position that comes
    /// in, are called direct feedthrough; the master resolves them, see CoSimulation.
    ///
    virtual std::vector<real_t> outputs(const std::vector<real_t> &inputs) const = 0;

    ///
    /// \brief Advance from t to t + H, the inputs moving linearly from u_begin to u_end
    ///
    virtual void do_step(real_t t, real_t H, const std::vector<real_t> &u_begin,
                         const std::vector<real_t> &u_end) = 0;

    ///
    /// \brief Remember the present state
    ///
    virtual void save() = 0;

    ///
    /// \brief Go back to the state save() remembered
    ///
    virtual void restore() = 0;
};

///
/// \brief A subsystem given by its equations: the state x moves with dx/dt = f(t, x, u), and
/// the outputs are y = h(x, u).
///
/// do_step() takes n_substeps steps of fourth order Runge-Kutta per macro step, with the inputs
/// interpolated linearly within the step.
///
class OdeSubsystem final : public CoSimSubsystem
{
  public:
    typedef std::function<std::vector<real_t>(real_t, const std::vector<real_t> &,
                                              const std::vector<real_t> &)>
        rhs_type;
    typedef std::function<std::vector<real_t>(const std::vector<real_t> &,
                                              const std::vector<real_t> &)>
        output_type;

    ///
    /// \param x0 the initial state
    /// \param n_inputs how many inputs f and h take
    /// \param n_outputs how many outputs h gives
    /// \param f the right-hand side, dx/dt = f(t, x, u)
    /// \param h the outputs, y = h(x, u)
    /// \param n_substeps the Runge-Kutta steps per macro step
    ///
    OdeSubsystem(std::vector<real_t> x0, uint_t n_inputs, uint_t n_outputs, rhs_type f,
                 output_type h, uint_t n_substeps = 10);

    uint_t n_inputs() const override { return n_inputs_; }

    uint_t n_outputs() const override { return n_outputs_; }

    std::vector<real_t> outputs(const std::vector<real_t> &inputs) const override;

    void do_step(real_t t, real_t H, const std::vector<real_t> &u_begin,
                 const std::vector<real_t> &u_end) override;

    void save() override { saved_ = x_; }

    void restore() override { x_ = saved_; }

    const std::vector<real_t> &state() const noexcept { return x_; }

    uint_t n_substeps() const noexcept { return n_substeps_; }

  private:
    std::vector<real_t> x_;
    std::vector<real_t> saved_;
    uint_t n_inputs_;
    uint_t n_outputs_;
    rhs_type f_;
    output_type h_;
    uint_t n_substeps_;
};

} // namespace cosim
} // namespace bitsim

#endif // BITSIM_COSIM_COSIM_SUBSYSTEM_H
