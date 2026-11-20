#include "bitsim/utils/io/csv_file_reader.h"
#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/utils/io/file_formats.h"

#include <boost/algorithm/string.hpp>

#include <stdexcept>

namespace bitsim
{
namespace utils::io
{

CSVFileReader::CSVFileReader(const std::string &file_name, const std::string delimeter)
    : FileReaderBase(file_name, FileFormats::Type::CSV), delimeter_(delimeter), current_row_idx_(0)
{
}

CSVFileReader::~CSVFileReader() { close(); }

std::vector<std::string> CSVFileReader::read_line()
{

    if (!this->is_open())
    {
        throw std::logic_error("Attempt to read from a file that is not open");
    }

    auto &f = this->get_file_stream();

    // An empty result at the end of the file, also when the last line ends with a newline.
    // Lines that start with # are comments, such as the header and the column names that
    // CSVWriter writes, and are skipped
    std::vector<std::string> result;
    std::string line;
    do
    {
        if (!std::getline(f, line))
        {
            return result;
        }
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }
    } while (!line.empty() && line.front() == '#');

    boost::algorithm::split(result, line, boost::is_any_of(delimeter_));
    current_row_idx_++;
    return result;
}

std::vector<uint_t> CSVFileReader::read_line_as_uint()
{

    auto line = read_line();
    std::vector<uint_t> line_int(line.size());

    for (uint_t i = 0; i < line.size(); ++i)
    {
        line_int[i] = std::stoull(line[i]);
    }

    return line_int;
}

} // namespace utils::io
} // namespace bitsim
