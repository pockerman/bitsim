#ifndef PREDICATES_H
#define PREDICATES_H

#include "bitsim/bitsim_types.h"

namespace bitsim
{

struct NotNull
{

    NotNull() {}

    template <typename ITERATOR> bool operator()(const ITERATOR *itr) const
    {
        return itr != nullptr;
    }
};

struct IsActive
{

    IsActive() {}
    template <typename ITERATOR> bool operator()(const ITERATOR *itr) const
    {
        return itr->is_active();
    }
};

struct ActiveBoundaryObject
{

    ActiveBoundaryObject() {}

    template <typename ITERATOR> bool operator()(const ITERATOR *itr) const
    {

        return (itr->is_active() && itr->on_boundary());
    }
};

} // namespace bitsim
#endif
