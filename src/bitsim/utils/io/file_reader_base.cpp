#include "bitsim/utils/io/file_reader_base.h"
#include "bitsim/bitsim_config.h"

#ifdef BITSIM_LOG
#include "bitsim/utils/logger.h"
#endif

namespace bitsim
{
namespace utils::io
{

FileReaderBase::FileReaderBase(const std::string &file_name, FileFormats::Type t)
    : FileHandlerBase<std::ifstream>(file_name, t)
{
}

void FileReaderBase::open()
{

    auto file_name = this->get_filename();

#ifdef BITSIM_LOG
    BITSIM_LOG_INFO << "Reading file: " << file_name;
    ;
#endif

    auto &f = this->get_file_stream();

    if (!f.is_open())
    {

        try
        {
            f.open(file_name, std::ios_base::in);

#ifdef BITSIM_DEBUG

            if (!f.good())
            {
                std::string msg = "Failed to open file: " + file_name;
                assert(false && msg.c_str());
            }
#endif
        }
        catch (...)
        {

#ifdef BITSIM_DEBUG
            std::string msg("Failed to open file: ");
            msg += file_name;
            assert(false && msg.c_str());
#else
            throw;
#endif
        }
    }
}

} // namespace utils::io
} // namespace bitsim
