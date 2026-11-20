#include "bitsim/rigid_bodies/body_translation.h"

namespace bitsim
{
namespace rigid_bodies
{

std::ostream &RBTranslation::print(std::ostream &out) const noexcept
{
    out << x << ", " << y << ", " << z << std::endl;
    return out;
}

} // namespace rigid_bodies
} // namespace bitsim