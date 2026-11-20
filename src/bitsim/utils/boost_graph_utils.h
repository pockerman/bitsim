#ifndef BOOST_GRAPH_UTILS_H
#define BOOST_GRAPH_UTILS_H

#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"

namespace bitsim
{
namespace utils
{

template <typename GraphType, typename Predicate>
uint_t find_vertex(const GraphType &graph, const Predicate &pred)
{

    for (uint_t v = 0; v < graph.n_vertices(); ++v)
    {
        const auto &vertex = graph.get_vertex(v);
        if (pred(vertex))
        {
            return vertex.id;
        }
    }

    return bitsim::consts::INVALID_ID;
}

} // namespace utils
} // namespace bitsim
#endif // BOOST_GRAPH_UTILS_H
