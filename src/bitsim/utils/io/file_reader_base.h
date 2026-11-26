// SPDX-FileCopyrightText: 2024 <copyright holder> <email>
// SPDX-License-Identifier: Apache-2.0

#ifndef BITSIM_UTILS_IO_FILE_READER_BASE_H
#define BITSIM_UTILS_IO_FILE_READER_BASE_H

#include "bitsim/utils//io/file_formats.h"
#include "bitsim/utils//io/file_handler_base.h"

#include <fstream>
#include <string>

namespace bitsim
{
namespace utils
{
namespace io
{
/**
 * @todo write docs
 */
class FileReaderBase : public FileHandlerBase<std::ifstream>
{
  public:
    virtual ~FileReaderBase() = default;

    ///
    /// \brief Attempts to open the file for reading
    ///
    virtual void open() override;

    ///
    /// \brief Returns true if the underlying stream
    /// handler has reached the EOF
    ///
    bool eof() const { return this->get_file_stream().eof(); }

  protected:
    ///
    /// \brief Constructor
    ///
    FileReaderBase(const std::string &file_name, FileFormats::Type t);
};

} // namespace io
} // namespace utils
} // namespace bitsim
#endif // BITSIM_UTILS_IO_FILE_READER_BASE_H
