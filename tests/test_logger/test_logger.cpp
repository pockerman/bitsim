// The logger: levels, what reaches a file sink and what is never formatted
#include "bitsim/utils/logger.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace
{
using bitsim::log::Level;

std::vector<std::string> read_lines(const std::string &filename)
{
    bitsim::log::flush();
    std::ifstream file(filename);
    std::vector<std::string> lines;
    for (std::string line; std::getline(file, line);)
    {
        lines.push_back(line);
    }
    return lines;
}

// one file for all the tests: a sink cannot be removed
const std::string &log_file()
{
    static const std::string filename = []
    {
        const auto path = std::filesystem::temp_directory_path() / "bitsim_test_logger.log";
        bitsim::log::add_file_sink(path.string(), true);

        // the level and the message only, to compare whole lines
        bitsim::log::set_pattern("[%n] [%l] %v");
        return path.string();
    }();
    return filename;
}

int counter = 0;
int count() { return ++counter; }
} // namespace

TEST(TestLogger, LevelsPerLoggerAndForAll)
{
    bitsim::log::set_level(Level::info);
    EXPECT_EQ(bitsim::log::get_level("bitsim"), Level::info);

    bitsim::log::set_level("test_levels", Level::error);
    EXPECT_EQ(bitsim::log::get_level("test_levels"), Level::error);
    EXPECT_EQ(bitsim::log::get_level("bitsim"), Level::info);

    // setting all of them sets those made already and those made later
    bitsim::log::set_level(Level::warn);
    EXPECT_EQ(bitsim::log::get_level("test_levels"), Level::warn);
    EXPECT_EQ(bitsim::log::get_level("bitsim"), Level::warn);
    EXPECT_EQ(bitsim::log::get_level("test_made_later"), Level::warn);

    bitsim::log::set_level(Level::info);
}

TEST(TestLogger, MessagesBelowTheLevelAreNotFormatted)
{
    bitsim::log::set_level("bitsim", Level::warn);
    counter = 0;
    BITSIM_LOG_INFO << "not written " << count();
    BITSIM_LOG_DEBUG << "not written " << count();
    EXPECT_EQ(counter, 0);

    BITSIM_LOG_WARN << "written " << count();
    EXPECT_EQ(counter, 1);
    bitsim::log::set_level("bitsim", Level::info);
}

TEST(TestLogger, WritesToTheFileSink)
{
    const auto &filename = log_file();
    const auto before = read_lines(filename).size();

    bitsim::log::set_level("bitsim", Level::info);
    BITSIM_LOG_INFO << "Reading file: "
                    << "robot.json";
    BITSIM_LOG_WARN << "x = " << 1.5 << std::endl;
    BITSIM_LOG_TO("test_other", Level::error) << "from another logger";
    BITSIM_LOG_DEBUG << "below the level";

    const auto lines = read_lines(filename);
    ASSERT_EQ(lines.size(), before + 3);
    EXPECT_EQ(lines[before], "[bitsim] [info] Reading file: robot.json");

    // the new line of std::endl is not written
    EXPECT_EQ(lines[before + 1], "[bitsim] [warning] x = 1.5");
    EXPECT_EQ(lines[before + 2], "[test_other] [error] from another logger");
}

TEST(TestLogger, WorksInAnIfWithoutBraces)
{
    const auto &filename = log_file();
    const auto before = read_lines(filename).size();

    // the macro is an if-else, so it must not take the else of the caller
    bool taken = false;
    if (false)
        BITSIM_LOG_WARN << "not written";
    else
        taken = true;

    EXPECT_TRUE(taken);
    EXPECT_EQ(read_lines(filename).size(), before);
}
