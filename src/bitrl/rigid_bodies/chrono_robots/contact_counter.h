#ifndef BITRL_RIGID_BODIES_CHRONO_ROBOTS_CONTACT_COUNTER_H
#define BITRL_RIGID_BODIES_CHRONO_ROBOTS_CONTACT_COUNTER_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_types.h"

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChContactContainer.h>
#include <chrono/physics/ChSystem.h>

#include <memory>
#include <vector>

namespace bitrl
{
namespace rb::bitrl_chrono
{

/**
 * @class ContactCounter
 * @ingroup rb_chrono
 * @brief Counts the contacts that touch any of a set of bodies, such as the obstacles a robot
 * should not hit.
 *
 * Call count() after each step of the system: it asks the system for its contacts and returns
 * how many of them touch one of the bodies and are in contact, with the bodies overlapping.
 */
class ContactCounter : public chrono::ChContactContainer::ReportContactCallback
{
  public:
    explicit ContactCounter(std::vector<std::shared_ptr<chrono::ChBody>> bodies);

    ///
    /// @brief The contacts of the system that touch one of the bodies now
    ///
    uint_t count(chrono::ChSystem &system);

    bool OnReportContact(const chrono::ChVector3d &, const chrono::ChVector3d &,
                         const chrono::ChMatrix33<> &, double distance, double,
                         const chrono::ChVector3d &, const chrono::ChVector3d &,
                         chrono::ChContactable *a, chrono::ChContactable *b, int) override;

  private:
    bool is_watched_(const chrono::ChContactable *c) const;

    std::vector<std::shared_ptr<chrono::ChBody>> bodies_;
    uint_t n_contacts_{0};
};

} // namespace rb::bitrl_chrono
} // namespace bitrl

#endif // BITRL_CHRONO
#endif // BITRL_RIGID_BODIES_CHRONO_ROBOTS_CONTACT_COUNTER_H
