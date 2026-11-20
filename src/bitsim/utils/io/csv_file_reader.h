#ifndef CSV_FILE_READER_H
#define CSV_FILE_READER_H

#include "bitsim/bitsim_types.h"
#include "bitsim/utils/io/file_reader_base.h"

#include <string>
#include <vector>

namespace bitsim
{
namespace utils
{
namespace io
{

///
/// \brief The CSVFileReader class
///
class CSVFileReader : public FileReaderBase
{
  public:
    ///
    /// \brief The default column delimiter
    ///
    static std::string default_delimeter() { return ","; }

    ///
    /// \brief Constructor
    ///
    CSVFileReader(const std::string &file_path,
                  const std::string delimeter = CSVFileReader::default_delimeter());

    ///
    /// \brief Destructor. Close the file if it is empty
    ///
    ~CSVFileReader();

    ///
    /// \brief Read a line from the file, split at the delimiter. The file must be open. Lines
    /// that start with # are comments and are skipped, such as the header and the column names
    /// that CSVWriter writes. At the end of the file the result is empty
    ///
    std::vector<std::string> read_line();

    ///
    /// \brief read_line_as_uint. Read the line
    /// as uint
    ///
    std::vector<uint_t> read_line_as_uint();

  private:
    ///
    /// \brief The delimiter
    ///
    const std::string delimeter_;

    ///
    /// \brief The current row index
    ///
    uint_t current_row_idx_;
};

} // namespace io
} // namespace utils
} // namespace bitsim
#endif // CSV_FILE_READER_H
