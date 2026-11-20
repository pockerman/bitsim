#include "bitsim/bitsim_config.h"

#ifdef BITSIM_CHRONO

#include "bitsim/sensors/backends/chrono_infrared_backend.h"

#include <algorithm>
#include <stdexcept>

namespace bitsim
{
namespace sensors::backends
{

const std::string CHRONO_InfraredBackend::BACKEND_TYPE = "SIMULATION";

CHRONO_InfraredBackend::CHRONO_InfraredBackend() : InfraredBackendBase(BACKEND_TYPE) {}

CHRONO_InfraredBackend::CHRONO_InfraredBackend(std::shared_ptr<chrono::ChBody> body, Floor floor)
    : InfraredBackendBase(BACKEND_TYPE), floor_(std::move(floor))
{
    set_chrono_body(body);
}

std::vector<real_t> CHRONO_InfraredBackend::read_values()
{
    if (!body_)
    {
        throw std::logic_error("CHRONO_InfraredBackend has no body: call set_chrono_body()");
    }
    if (!floor_)
    {
        throw std::logic_error("CHRONO_InfraredBackend has no floor: call set_floor()");
    }

    const auto &frame = body_->GetFrameRefToAbs();
    std::vector<real_t> values;
    values.reserve(n_elements());
    for (const auto &e : elements())
    {
        const auto p = frame.TransformPointLocalToParent(chrono::ChVector3d(e[0], e[1], e[2]));
        values.push_back(std::clamp(floor_(p.x(), p.y()), 0.0, 1.0));
    }
    return values;
}

} // namespace sensors::backends
} // namespace bitsim

#endif
