#ifndef BITSIM_UTILS_BITSIM_UTILS_H
#define BITSIM_UTILS_BITSIM_UTILS_H

#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <string>

namespace bitsim
{
namespace utils
{
/**
 * Generate UUID4 strings using Boost::uuids.
 * For more info on UUIDs see: https://www.ibm.com/docs/en/cobol-zos/6.3.0?topic=functions-uuid4
 * @return
 */
inline std::string uuid4() { return boost::uuids::to_string(boost::uuids::random_generator()()); }

} // namespace utils
} // namespace bitsim

#endif // BITSIM_UTILS_BITSIM_UTILS_H
