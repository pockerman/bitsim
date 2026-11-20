#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_config.h"

#include <cstdlib>
#include <system_error>

namespace bitsim
{
namespace consts
{

std::filesystem::path data_dir()
{
    if (const char *from_env = std::getenv("BITSIM_DATA_DIR"); from_env && *from_env)
    {
        return std::filesystem::path(from_env);
    }

    std::error_code error;
    const std::filesystem::path source_dir(BITSIM_SOURCE_DATA_DIR);
    if (std::filesystem::is_directory(source_dir / "robots", error))
    {
        return source_dir;
    }

    return std::filesystem::path(BITSIM_INSTALL_DATA_DIR);
}

} // namespace consts
} // namespace bitsim
