#ifndef BITSIM_RIGID_BODIES_BODY_TRANSLATION_H
#define BITSIM_RIGID_BODIES_BODY_TRANSLATION_H

#include "bitsim/bitsim_types.h"

#include <initializer_list>
#include <ostream>
#include <stdexcept>

namespace bitsim
{
namespace rigid_bodies
{

///
/// \brief struct RBTranslation utility struct representing
//// the translation of a rigid body
///
struct RBTranslation
{
    real_t x;

    real_t y;

    real_t z;

    ///
    /// \brief RBTranslation. Default constructor
    ///
    RBTranslation();
    RBTranslation(std::initializer_list<real_t> vals);

    ///
    /// \brief print
    /// \param out
    /// \return
    ///
    std::ostream &print(std::ostream &out) const noexcept;
};

inline RBTranslation::RBTranslation() : RBTranslation({0.0, 0.0, 0.0}) {}

inline RBTranslation::RBTranslation(std::initializer_list<real_t> vals) : x(0.0), y(0.0), z(0.0)
{

    if (vals.size() != 3)
    {
        throw std::invalid_argument("RBTranslation needs 3 values, x, y and z");
    }
    auto v = vals.begin();
    x = *v;
    v++;
    y = *v;
    v++;
    z = *v;
}

inline std::ostream &operator<<(std::ostream &out, const RBTranslation &info)
{
    return info.print(out);
}

} // namespace rigid_bodies
} // namespace bitsim

#endif // BITSIM_RIGID_BODIES_BODY_TRANSLATION_H
