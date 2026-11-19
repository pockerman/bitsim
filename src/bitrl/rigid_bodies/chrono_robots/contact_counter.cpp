#include "bitrl/rigid_bodies/chrono_robots/contact_counter.h"

#ifdef BITRL_CHRONO

#include <algorithm>

namespace bitrl
{
namespace rb::bitrl_chrono
{

ContactCounter::ContactCounter(std::vector<std::shared_ptr<chrono::ChBody>> bodies)
    : bodies_(std::move(bodies))
{
}

uint_t ContactCounter::count(chrono::ChSystem &system)
{
    n_contacts_ = 0;
    // The container keeps the callback only while it reports, so a non-owning pointer is enough
    std::shared_ptr<ReportContactCallback> self(this, [](ReportContactCallback *) {});
    system.GetContactContainer()->ReportAllContacts(self);
    return n_contacts_;
}

bool ContactCounter::OnReportContact(const chrono::ChVector3d &, const chrono::ChVector3d &,
                                     const chrono::ChMatrix33<> &, double distance, double,
                                     const chrono::ChVector3d &, const chrono::ChVector3d &,
                                     chrono::ChContactable *a, chrono::ChContactable *b, int)
{
    if (distance < 0.0 && (is_watched_(a) || is_watched_(b)))
    {
        ++n_contacts_;
    }
    return true;
}

bool ContactCounter::is_watched_(const chrono::ChContactable *c) const
{
    return std::any_of(bodies_.begin(), bodies_.end(),
                       [c](const auto &body)
                       { return static_cast<const chrono::ChContactable *>(body.get()) == c; });
}

} // namespace rb::bitrl_chrono
} // namespace bitrl

#endif // BITRL_CHRONO
