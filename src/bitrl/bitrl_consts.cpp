#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_config.h"

#include <cstdlib>
#include <system_error>

namespace bitrl
{
namespace consts
{

std::filesystem::path data_dir()
{
    if (const char *from_env = std::getenv("BITRL_DATA_DIR"); from_env && *from_env)
    {
        return std::filesystem::path(from_env);
    }

    std::error_code error;
    const std::filesystem::path source_dir(BITRL_SOURCE_DATA_DIR);
    if (std::filesystem::is_directory(source_dir / "robots", error))
    {
        return source_dir;
    }

    return std::filesystem::path(BITRL_INSTALL_DATA_DIR);
}

} // namespace consts
} // namespace bitrl
