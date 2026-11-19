#include "bitrl/bitrl_types.h"
#include "bitrl/utils/io/csv_file_reader.h"
#include "bitrl/utils/io/csv_file_writer.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using bitrl::uint_t;
using bitrl::utils::io::CSVFileReader;
using bitrl::utils::io::CSVWriter;

std::string temp_file(const std::string &name)
{
    return (std::filesystem::temp_directory_path() / ("bitrl_test_" + name + ".csv")).string();
}
} // namespace

TEST(TestCSVFileReader, ReadsWhatCSVWriterWrote)
{
    const auto path = temp_file("roundtrip");
    {
        CSVWriter writer(path);
        writer.open();
        writer.write_column_names(std::vector<std::string>{"a", "b", "c"});
        writer.write_row(std::vector<uint_t>{1, 2, 3});
        writer.write_row(std::vector<uint_t>{40, 50, 60});
        writer.close();
    }

    // CSVWriter writes its header and the column names as comments, # lines, which the reader
    // skips
    CSVFileReader reader(path);
    reader.open();
    EXPECT_EQ(reader.read_line_as_uint(), (std::vector<uint_t>{1, 2, 3}));
    EXPECT_EQ(reader.read_line_as_uint(), (std::vector<uint_t>{40, 50, 60}));
    // The file ends with a newline: the end gives an empty line, not {""}
    EXPECT_TRUE(reader.read_line().empty());
    EXPECT_TRUE(reader.read_line_as_uint().empty());
    std::filesystem::remove(path);
}

TEST(TestCSVFileReader, OtherDelimitersAndWindowsLineEnds)
{
    const auto path = temp_file("delimiter");
    {
        std::ofstream out(path, std::ios::binary);
        out << "# a comment\r\nx;y\r\n1;2\r\n";
    }
    CSVFileReader reader(path, ";");
    reader.open();
    EXPECT_EQ(reader.read_line(), (std::vector<std::string>{"x", "y"}));
    EXPECT_EQ(reader.read_line_as_uint(), (std::vector<uint_t>{1, 2}));
    EXPECT_TRUE(reader.read_line().empty());
    std::filesystem::remove(path);
}

TEST(TestCSVFileReader, ReadingAClosedFileThrows)
{
    CSVFileReader reader(temp_file("closed"));
    EXPECT_THROW(reader.read_line(), std::logic_error);
}
